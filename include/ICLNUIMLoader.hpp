#pragma once

#include <vector>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Geometry>


struct ICLNUIMLoader
{
    Observation allObservations()
    {
        Observation obs;
        for (auto &frame : frames)
            obs.append(frame);
        return obs;
    }

    std::vector<Observation> frames;

    int img_width{640};
    int img_height{480};

    float focal_x{-640}; //{481.2};
    float focal_y{481.2}; //{-640};
    float u0{319.5};
    float v0{239.5};

    ICLNUIMLoader() = default;

    bool load(const std::string &foldername, int trajectory, size_t numberOfSamples,
              int sampleSparsification = 10, int pixelSparsification = 10)
    {
        std::vector<float> depth_array(img_width*img_height, 0);
        _foldername = foldername;
        frames.clear();

        Eigen::Matrix4f postTransformation;
        // post transformation matrices come from
        // https://github.com/mp3guy/SurfReg/blob/master/src/SurfReg.cpp
        if (trajectory == 0)
        {
            postTransformation << 0.999759, -0.000287637, 0.0219655, 0, /*-1.36022,*/
                    0.000160294, 0.999983, 0.00579897, 0, /*1.48382,*/
                    0.0219668, 0.00579404, -0.999742, 0, /*1.44256,*/
                    0, 0, 0, 1;
            _foldername += "/traj0";
        }
        else if (trajectory == 1)
        {
            postTransformation << 0.99975, -0.00789018, 0.0209474, 0.0133734,
                    0.00789931, 0.999969, -0.000353282, 0, /*1.27618,*/
                    0.0209439, -0.000518671, -0.999781, 0.0376324,
                    0, 0, 0, 1;
            _foldername += "/traj1";
        }
        else if (trajectory == 2)
        {
            postTransformation << 0.999822, 0.0034419, 0.0185526, 0, /*-0.786316,*/
                    -0.00350915, 0.999987, 0.00359374, 0, /*1.28433,*/
                    0.01854, 0.00365819, -0.999821, 0, /*1.45583,*/
                    0, 0, 0, 1;
            _foldername += "/traj2";
        }
        else if (trajectory == 3)
        {
            postTransformation << 0.999778, -0.000715914, 0.0210893, 0, /*-1.13311,*/
                    0.000583688, 0.99998, 0.0062754, 0, /*1.26825,*/
                    0.0210934, 0.0062617, -0.999758, 0, /*0.901866,*/
                    0, 0, 0, 1;
            _foldername += "/traj3";
        }

        std::string filename = foldername + "/livingRoom0.gt.freiburg.txt";
        std::cout << "Loading trajectory " << trajectory << " from " << filename << "..." << std::endl;

        std::vector<Eigen::Vector3f> positions;
        std::vector<Eigen::Quaternionf> orientations;
        _readTrajectory(filename, positions, orientations);
        for (int i = 0; i < numberOfSamples; i += sampleSparsification)
        {
            if (!_readDepth(i, 0, depth_array))
                return !frames.empty();

            Eigen::Matrix4f pose = postTransformation * _readPose(i, 0);

            Eigen::Vector3f position = pose.block(0,3,3,1); //(postTransformation * pose.block(0,3,4,1)).block(0,0,3,1); //pose.block(0,3,3,1) + postTransformation.block(0,3,3,1);
            Eigen::Matrix3f orientation = pose.block(0,0,3,3); //postTransformation.block(0,0,3,3) * pose.block(0,0,3,3);

            Eigen::Vector3f target = orientation * Eigen::Vector3f::UnitZ();

            Observation frame;
            for (int v = 0; v < img_height; v += pixelSparsification)
            {
                for (int u = img_width/2-100; u < img_width/2+100; u += pixelSparsification)
                {
                    float u_u0_by_fx = (-u + u0) / focal_x;
                    float v_v0_by_fy = (v - v0) / focal_y;

                    float depth = depth_array[u + (img_height-v-1) * img_width];
                    float z = depth / std::sqrt(u_u0_by_fx * u_u0_by_fx +
                                                v_v0_by_fy * v_v0_by_fy + 1);

                    Eigen::Vector3f ltarget;
                    ltarget[0] = (u_u0_by_fx) * (z);
                    ltarget[1] = (v_v0_by_fy) * (z);
                    ltarget[2] = z;
//                    ltarget[3] = 0;
                    ltarget.normalize();
//                    ltarget[3] = 1;
//                    std::cout << "ltarget:  " << ltarget.transpose() << std::endl;
//                    target[3] = 1;
//                    ltarget = pose * ltarget;
                    ltarget = orientation * ltarget;
//                    std::swap(ltarget[0], ltarget[1]);
//                    ltarget[0] = -ltarget[0];
//                    std::cout << "ltargeto: " << ltarget.transpose() << std::endl;

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

//                    position += postTransformation.block(0, 3, 3, 1);
//                    ltarget = (postTransformation.block(0, 0, 3, 3) * ltarget);

//                    SensorRay sensor(
//                            Parameters::Vec3Type(pose(0, 3), pose(1, 3), pose(2, 3)),
//                            Parameters::Vec3Type(orientation[0], orientation[1], orientation[2]),
//                            depth);
                    SensorRay lsensor(
//                            Parameters::Vec3Type(endpoint[1]-2.82, endpoint[0]+0.23, endpoint[2]),
                            Parameters::Vec3Type(-position[0], position[1], -position[2]),
                            Parameters::Vec3Type(-ltarget[0], ltarget[1], -ltarget[2]),
                            depth);
                    frame.append(Measurement::voxel(lsensor, depth));
                }
            }
            frames.push_back(frame);
            if (frames.size() % 20 == 0)
                std::cout << ".";
            std::flush(std::cout);
        }
        std::cout << std::endl;
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

//        std::cout << "DEPTH from " << depthFileName << std::endl;
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

        Eigen::Vector3f direction(3);
        Eigen::Vector3f upvector(3);
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
//                direction[3] = 0.0f;
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
//                upvector[3] = 0.0f;
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
        Eigen::Vector3f x = upvector.cross(z);
        x.normalize();

        /// y = cross(z,x)
        Eigen::Vector3f y = z.cross(x);

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
//        std::cout << std::endl << "POSE from " << text_file_name << ":" << std::endl;
//        std::cout << transformation << std::endl;
        return transformation;
    }
};
