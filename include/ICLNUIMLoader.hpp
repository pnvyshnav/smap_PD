#pragma once

#include <vector>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Geometry>


struct ICLNUIMLoader
{
    Observation observation{Observation()};

    int img_width{640};
    int img_height{480};

    float focal_x{481.2};
    float focal_y{-480};
    float u0{319.5};
    float v0{239.5};

    ICLNUIMLoader() = default;

    bool load(const std::string &foldername, size_t numberOfSamples)
    {
        std::vector<float> depth_array(img_width*img_height, 0);
        _foldername = foldername;
        observation.clear();

        std::vector<Eigen::Vector3f> positions;
        std::vector<Eigen::Quaternionf> orientations;
        _readTrajectory(foldername + "/livingRoom0.gt.freiburg.txt", positions, orientations);

        for (int i = 0; i < numberOfSamples*15; i += 15)
        {
            if (!_readDepth(i, 0, depth_array))
                return false;
//
//            auto pose = _readPose(i, 0);
//            Eigen::Vector3f zAxis(0.f, 0.f, 1.f);
//            Eigen::Vector3f orientation;

            auto position = positions[i];
            auto orientation = orientations[i];

//            auto target = orientation * Eigen::Vector3f(-1,0,-1);
//
//            SensorRay sensor(
////                            Parameters::Vec3Type(endpoint[1]-2.82, endpoint[0]+0.23, endpoint[2]),
//                    Parameters::Vec3Type(position[0], position[1]+1.51739, -position[2]),
//                    Parameters::Vec3Type(target[0], target[1], -target[2]),
//                    1);
//            observation.append(Measurement::voxel(sensor, 1));
            std::cout << "position: " << position.transpose() << std::endl;
//            std::cout << "target:   " << target.transpose() << std::endl;
            std::cout << "orientation: "
                      << orientation.x() << " "
                      << orientation.y() << " "
                      << orientation.z() << " "
                      << orientation.w() << std::endl << std::endl;

            for (int v = 0; v < img_height; v += 80) // TODO note skipping
            {
                for (int u = 0; u < img_width; u += 80) // TODO note skipping
                {
                    float u_u0_by_fx = (u - u0) / focal_x;
                    float v_v0_by_fy = (v - v0) / focal_y;

                    float depth = depth_array[u + v * img_width];
                    float z = depth / std::sqrt(u_u0_by_fx * u_u0_by_fx +
                                                v_v0_by_fy * v_v0_by_fy + 1);

                    Eigen::Vector3f target;
                    target[0] = -(u_u0_by_fx) * (z);
                    target[1] = (v_v0_by_fy) * (z);
                    target[2] = -z;
//                    target[3] = 1;
                    target = orientation * target;
//                    orientation = target;

//                    Eigen::Matrix3f rotation = pose.block(0, 0, 3, 3);
//                    Eigen::Vector3f translation = pose.block(0, 3, 3, 1);

//                    Eigen::Vector3f endpoint;
////                    endpoint = rotation * target + translation;
//                    endpoint = (pose * target).block(0,0,3,1);
//                    Eigen::Vector3f orientation = rotation * target.block(0,0,3,1);

//                    std::cout << "Orientation: " << target.transpose() << std::endl;
//                    std::cout << "Position:    " << pose.block(0, 3, 3, 1).transpose() << std::endl;
//                    std::cout << "Endpoint:    " << endpoint.transpose() << std::endl;
//
//                    Eigen::Vector3f position = pose.block(0, 3, 3, 1).transpose();
//                    std::swap(position[0], position[1]);
//                    orientation = (endpoint-position).normalized();

//                    SensorRay sensor(
//                            Parameters::Vec3Type(pose(0, 3), pose(1, 3), pose(2, 3)),
//                            Parameters::Vec3Type(orientation[0], orientation[1], orientation[2]),
//                            depth);
                    SensorRay sensor(
//                            Parameters::Vec3Type(endpoint[1]-2.82, endpoint[0]+0.23, endpoint[2]),
                            Parameters::Vec3Type(position[0], position[1]+1.51739, position[2]),
                            Parameters::Vec3Type(target[0], target[1], -target[2]),
                            depth);
                    observation.append(Measurement::voxel(sensor, depth));
                }
                std::cout << std::endl;
            }
        }
        return true;
    }

private:
    std::string _foldername;

    bool _readTrajectory(std::string filename,
                         std::vector<Eigen::Vector3f> &positions,
                         std::vector<Eigen::Quaternionf> &orientations)
    {
        std::ifstream tfile;
        tfile.open(filename);
        if (!tfile || tfile.bad())
            return false;

        int frame;
        float px, py, pz, qx, qy, qz, qw;
        while (tfile >> frame >> px >> py >> pz >> qx >> qy >> qz >> qw)
        {
            positions.push_back(Eigen::Vector3f(px, py, pz));
            orientations.push_back(Eigen::Quaternionf(qw, qx, qy, qz));
        }
        tfile.close();
        return true;
    }

    bool _readDepth(int ref_img_no, int which_blur_sample, std::vector<float> &depth_array)
    {
        if (depth_array.empty())
            depth_array = std::vector<float>(img_width*img_height, 0);

        char depthFileName[300];

        sprintf(depthFileName, "%s/scene_%02d_%04d.depth",
                _foldername.c_str(), which_blur_sample, ref_img_no);

        std::ifstream depthfile;
        depthfile.open(depthFileName);

        if (!depthfile || depthfile.bad())
        {
            std::cerr << "Error reading file " << depthFileName << std::endl;
            return false;
        }

        for(int i = 0 ; i < img_height ; i++)
        {
            for (int j = 0 ; j < img_width ; j++)
            {
                float val = 0;
                depthfile >> val;
                depth_array[i*img_width+j] = val;
            }
        }

        depthfile.close();

        std::cout << "DEPTH from " << depthFileName << std::endl;
        return true;
    }

    Eigen::Matrix4f _readPose(int ref_img_no, int which_blur_sample)
    {
        char text_file_name[360];

        sprintf(text_file_name, "%s/scene_%02d_%04d.txt", _foldername.c_str(),
                which_blur_sample, ref_img_no);

        std::ifstream cam_pars_file(text_file_name);
        if (!cam_pars_file)
            std::cerr << "Error reading file " << text_file_name << std::endl;

        char readlinedata[300];

        Eigen::Vector4f direction(4);
        Eigen::Vector4f upvector(4);
        Eigen::Vector3f posvector(3);

        while (true)
        {
            cam_pars_file.getline(readlinedata, 300);

            if (cam_pars_file.eof())
                break;

            std::istringstream iss;

            if (strstr(readlinedata, "cam_dir") != nullptr)
            {
                std::string cam_dir_str(readlinedata);

                cam_dir_str = cam_dir_str.substr(cam_dir_str.find("= [") + 3);
                cam_dir_str = cam_dir_str.substr(0, cam_dir_str.find("]"));

                iss.str(cam_dir_str);
                iss >> direction[0];
                iss.ignore(1, ',');
                iss >> direction[1];
                iss.ignore(1, ',');
                iss >> direction[2];
                iss.ignore(1, ',');
                //cout << direction[0]<< ", "<< direction[1] << ", "<< direction[2] << endl;
                direction[3] = 0.0f;

            }

            if (strstr(readlinedata, "cam_up") != nullptr)
            {
                std::string cam_up_str(readlinedata);

                cam_up_str = cam_up_str.substr(cam_up_str.find("= [") + 3);
                cam_up_str = cam_up_str.substr(0, cam_up_str.find("]"));


                iss.str(cam_up_str);
                iss >> upvector[0];
                iss.ignore(1, ',');
                iss >> upvector[1];
                iss.ignore(1, ',');
                iss >> upvector[2];
                iss.ignore(1, ',');
                upvector[3] = 0.0f;

            }

            if (strstr(readlinedata, "cam_pos") != nullptr)
            {
                std::string cam_pos_str(readlinedata);

                cam_pos_str = cam_pos_str.substr(cam_pos_str.find("= [") + 3);
                cam_pos_str = cam_pos_str.substr(0, cam_pos_str.find("]"));

                iss.str(cam_pos_str);
                iss >> posvector[0];
                iss.ignore(1, ',');
                iss >> posvector[1];
                iss.ignore(1, ',');
                iss >> posvector[2];
                iss.ignore(1, ',');

            }

        }

        /// z = dir / norm(dir)
        Eigen::Vector3f z(3);
        z[0] = direction[0];
        z[1] = direction[1];
        z[2] = direction[2];
        z.normalize();

        /// x = cross(cam_up, z)
        Eigen::Vector3f x = Eigen::VectorXf::Zero(3);
        x[0] = upvector[0] * z[2] - upvector[0] * z[1];
        x[1] = upvector[1] * z[0] - upvector[1] * z[2];
        x[2] = upvector[2] * z[1] - upvector[2] * z[0];
        x.normalize();

        /// y = cross(z,x)
        Eigen::Vector3f y = Eigen::VectorXf::Zero(3);
        y[0] = z[1] * x[2] - z[2] * x[1];
        y[1] = z[2] * x[0] - z[0] * x[2];
        y[2] = z[0] * x[1] - z[1] * x[0];

        Eigen::Matrix3f R = Eigen::MatrixXf::Zero(3, 3);
        R(0,0) = x[0];
        R(1,0) = x[1];
        R(2,0) = x[2];

        R(0,1) = y[0];
        R(1,1) = y[1];
        R(2,1) = y[2];

        R(0,2) = z[0];
        R(1,2) = z[1];
        R(2,2) = z[2];

        Eigen::Matrix4f transformation(4, 4);
        transformation.block(0, 0, 3, 3) = R;
        transformation.block(0, 3, 3, 1) = posvector;
        transformation.block(3, 0, 1, 4) << 0, 0, 0, 1;
//        std::cout << std::endl << "R:" << std::endl;
//        std::cout << R << std::endl;
//        std::cout << std::endl << "upvector:" << std::endl;
//        std::cout << upvector << std::endl;
//        std::cout << std::endl << "direction:" << std::endl;
//        std::cout << direction << std::endl;
        std::cout << std::endl << "POSE from " << text_file_name << ":" << std::endl;
        std::cout << transformation << std::endl;
        return transformation; //TooN::SE3<>(R, posvector);
    }
};
