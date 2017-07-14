#include "../include/Drone.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/io/pcd_io.h>
//#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <depth_image_proc/depth_conversions.h>

#include <cmath>

#include "../include/Sensor.h"


#if INPUT_TYPE == INPUT_EUROC


//
// Calibration data from EuROC MAV dataset
//
const tf::Transform Drone::vicon(tf::Matrix3x3(
        0.33638, -0.01749,  0.94156,
        -0.02078, -0.99972, -0.01114,
        0.94150, -0.01582, -0.33665
), tf::Vector3(0.06901, -0.02781, -0.12395));
const tf::Transform Drone::camera(tf::Matrix3x3(
        0.0125552670891, -0.999755099723, 0.0182237714554,
        0.999598781151, 0.0130119051815, 0.0251588363115,
        -0.0253898008918, 0.0179005838253, 0.999517347078
), tf::Vector3(-0.0198435579556, 0.0453689425024, 0.00786212447038));

#elif INPUT_TYPE == INPUT_SNAPDRAGON
//
// Calibration data from Snapdragon Flight Stereo Camera
//
constexpr float focalX = 281.620143146f;
constexpr float focalY = 282.560278566f;
constexpr float principalPointX = 335.626405147f;
constexpr float principalPointY = 241.833393408f;
image_geometry::PinholeCameraModel snapdragonCameraModel;
sensor_msgs::CameraInfo snapdragonCameraInfo;

#endif

Drone::Drone() : _stopRequested(false)
{
#if INPUT_TYPE == INPUT_SNAPDRAGON
    snapdragonCameraInfo.K = { {
                            focalX, 0     , principalPointX
                          , 0     , focalY, principalPointY
                          , 0     , 0     , 1 }};
    snapdragonCameraModel.fromCameraInfo(snapdragonCameraInfo);
#endif
}


#if INPUT_TYPE == INPUT_EUROC
void Drone::handleMeasurements(PointsMessageConstPtr pointsMsg, TransformationMessageConstPtr transformationMsg)
{
    // convert messages to usable data types
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*pointsMsg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *pointCloud);

#ifndef PREPROCESSED_INPUT
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudFiltered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(pointCloud);
    sor.setLeafSize(Parameters::PointCloudResolution,
                    Parameters::PointCloudResolution,
                    Parameters::PointCloudResolution);
    sor.filter(*pointCloudFiltered);

    ROS_INFO("Point Cloud size reduced from %i to %i points.",
             (int)pointCloud->size(), (int)pointCloudFiltered->size());
#ifdef SKIP_LARGE_POINTCLOUDS
    if (pointCloudFiltered->size() > 1500)
    {
        ROS_WARN("SKIPPING POINT CLOUD (TOO LARGE).");
        return;
    }
#endif
    pointCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(pointCloudFiltered);



    tf::StampedTransform stampedTransform;
    tf::transformStampedMsgToTF(*transformationMsg, stampedTransform);
    tf::Transform transformation(stampedTransform);

    transformation *= vicon * camera;

//    //TODO broadcast tf and transformed PointCloud2 for debugging
//    static tf::TransformBroadcaster br;
//    auto time = ros::Time::now(); // uses simulation time from ros bag clock
//    tf::StampedTransform tr2(transformation, time, "map", "tf_drone");
//    br.sendTransform(tr2);
//    geometry_msgs::TransformStamped tfMsg;
//    tf::transformStampedTFToMsg(tr2, tfMsg);
//    _tfDronePub.publish(tfMsg);
//
//    sensor_msgs::PointCloud2 pointsTfMsg(*pointsMsg);
//    pointsTfMsg.header.frame_id = "tf_drone";
//    pointsTfMsg.header.stamp = time;
//    _tfPointCloudPub.publish(pointsTfMsg);
//
//    // publish downsampled point cloud
//    sensor_msgs::PointCloud2 downsampledPointsTfMsg;
//    pcl::toROSMsg(*pointCloudFiltered, downsampledPointsTfMsg);
//    downsampledPointsTfMsg.header.frame_id = "tf_drone";
//    downsampledPointsTfMsg.header.stamp = time;
//    _tfDownsampledPointCloudPub.publish(downsampledPointsTfMsg);
#else
    tf::StampedTransform stampedTransform;
    tf::transformStampedMsgToTF(*transformationMsg, stampedTransform);
    tf::Transform transformation(stampedTransform);
#endif

    Parameters::Vec3Type origin((float) transformation.getOrigin().x(),
                                (float) transformation.getOrigin().y(),
                                (float) transformation.getOrigin().z());

    tf::Vector3 orientation(1, 0, 0);
    orientation = transformation * orientation;

    _history.add(TrajectoryPoint(
            Eigen::Vector3f(origin.x(), origin.y(), origin.z()),
            Eigen::Vector3f(orientation.x(), orientation.y(), orientation.z())));

#ifdef LOG_DETAILS
    ROS_INFO("Sensor Position:  %.3f %.3f %.3f", transformation.getOrigin().x(), transformation.getOrigin().y(),
    transformation.getOrigin().z());
    //ROS_INFO("Sensor Direction: %.3f %.3f %.3f", direction.x(), direction.y(), direction.z());
#endif

    if (std::isnan(transformation.getOrigin().x()))
    {
        ROS_WARN("Measured transformation origin is invalid.");
        return;
    }

    auto firstValidPoint = pointCloud->begin();
    while (firstValidPoint != pointCloud->end() && std::isnan(firstValidPoint->x))
        ++firstValidPoint;
    if (firstValidPoint == pointCloud->end())
    {
        ROS_WARN("Measured point cloud did not contain any valid points.");
        return;
    }

#ifdef LOG_DETAILS
    float xmin = firstValidPoint->x;
    float ymin = firstValidPoint->y;
    float zmin = firstValidPoint->z;
    float xmax = firstValidPoint->x;
    float ymax = firstValidPoint->y;
    float zmax = firstValidPoint->z;
#endif

    unsigned int nans = 0;
    std::vector<Measurement> measurements;
    for (auto &p = firstValidPoint; p < pointCloud->end(); ++p)
    {
        if (std::isnan(p->x))
        {
            ++nans;
            continue;
        }

        tf::Vector3 point(p->x, p->y, p->z);
        point = transformation * point;
        point -= transformation.getOrigin();

#ifdef LOG_DETAILS
        xmin = std::min(xmin, (float)point.x());
        ymin = std::min(ymin, (float)point.y());
        zmin = std::min(zmin, (float)point.z());
        xmax = std::max(xmax, (float)point.x());
        ymax = std::max(ymax, (float)point.y());
        zmax = std::max(zmax, (float)point.z());
#endif

        //Parameters::Vec3Type pixel(point->x, point->y, point->z);
        Parameters::NumType measuredRange = point.length();
        point.normalize();
        Parameters::Vec3Type pixelDirection(point.x(), point.y(), point.z());
        SensorRay sensor(origin, pixelDirection, measuredRange); // TODO check Parameters::sensorRange
        measurements.push_back(Measurement::voxel(sensor, measuredRange));
    }
#ifdef LOG_DETAILS
    ROS_INFO("PointCloud");
    ROS_INFO("\tDimensions:");
    ROS_INFO("\t\tMin: %f %f %f", xmin, ymin, zmin);
    ROS_INFO("\t\tMax: %f %f %f", xmax, ymax, zmax);
    ROS_INFO("\tNaN points: %d of %d (%.2f%%)", (int)nans, (int)pointCloud->size(), (float)(nans*100./pointCloud->size()));
#endif
    Observation observation(measurements);
    publishObservation(observation);
}

#elif INPUT_TYPE == INPUT_SNAPDRAGON

void Drone::handleMeasurements(PointsMessageConstPtr depthImage, TransformationMessageConstPtr pose)
{
    sensor_msgs::ImageConstPtr imgPtr(depthImage);
    sensor_msgs::PointCloud2Ptr cloud2Ptr(new sensor_msgs::PointCloud2);
    depth_image_proc::convert<float>(imgPtr, cloud2Ptr, snapdragonCameraModel); // TODO set maximum range? Parameters::sensorRange

    // convert messages to usable data types
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud2Ptr, *pointCloud);

#ifndef PREPROCESSED_INPUT
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudFiltered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(pointCloud);
    sor.setLeafSize(Parameters::PointCloudResolution,
                    Parameters::PointCloudResolution,
                    Parameters::PointCloudResolution);
    sor.filter(*pointCloudFiltered);

    ROS_INFO("Point Cloud size reduced from %i to %i points.",
             (int)pointCloud->size(), (int)pointCloudFiltered->size());
#ifdef SKIP_LARGE_POINTCLOUDS
    if (pointCloudFiltered->size() > 1500)
    {
        ROS_WARN("SKIPPING POINT CLOUD (TOO LARGE).");
        return;
    }
#endif
    pointCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(pointCloudFiltered);

    auto position = pose->pose.position;
    auto orientation = pose->pose.orientation;

    tf::Transform transformation;
    transformation.setOrigin(tf::Vector3(position.x, position.y, position.z));
    transformation.setRotation(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));

    //TODO broadcast tf and transformed PointCloud2 for debugging
    static tf::TransformBroadcaster br;
    auto time = ros::Time::now(); // uses simulation time from ros bag clock
    tf::StampedTransform tr2(transformation, time, "map", "tf_drone");
    br.sendTransform(tr2);
    geometry_msgs::TransformStamped tfMsg;
    tf::transformStampedTFToMsg(tr2, tfMsg);
    _tfDronePub.publish(tfMsg);

    sensor_msgs::PointCloud2 pointsTfMsg(*cloud2Ptr);
    pointsTfMsg.header.frame_id = "tf_drone";
    pointsTfMsg.header.stamp = time;
    _tfPointCloudPub.publish(pointsTfMsg);

    // publish downsampled point cloud
    sensor_msgs::PointCloud2 downsampledPointsTfMsg;
    pcl::toROSMsg(*pointCloudFiltered, downsampledPointsTfMsg);
    downsampledPointsTfMsg.header.frame_id = "tf_drone";
    downsampledPointsTfMsg.header.stamp = time;
    _tfDownsampledPointCloudPub.publish(downsampledPointsTfMsg);
#else
    tf::StampedTransform stampedTransform;
    tf::transformStampedMsgToTF(*transformationMsg, stampedTransform);
    tf::Transform transformation(stampedTransform);
#endif

    Parameters::Vec3Type origin((float) transformation.getOrigin().x(),
                                (float) transformation.getOrigin().y(),
                                (float) transformation.getOrigin().z());

#ifdef LOG_DETAILS
    ROS_INFO("Sensor Position:  %.3f %.3f %.3f", transformation.getOrigin().x(), transformation.getOrigin().y(),
    transformation.getOrigin().z());
    //ROS_INFO("Sensor Direction: %.3f %.3f %.3f", direction.x(), direction.y(), direction.z());
#endif

    if (std::isnan(transformation.getOrigin().x()))
    {
        ROS_WARN("Measured transformation origin is invalid.");
        return;
    }

    auto firstValidPoint = pointCloud->begin();
    while (firstValidPoint != pointCloud->end() && std::isnan(firstValidPoint->x))
        ++firstValidPoint;
    if (firstValidPoint == pointCloud->end())
    {
        ROS_WARN("Measured point cloud did not contain any valid points.");
        return;
    }

#ifdef LOG_DETAILS
    float xmin = firstValidPoint->x;
    float ymin = firstValidPoint->y;
    float zmin = firstValidPoint->z;
    float xmax = firstValidPoint->x;
    float ymax = firstValidPoint->y;
    float zmax = firstValidPoint->z;
#endif

    unsigned int nans = 0;
    std::vector<Measurement> measurements;
    for (auto &p = firstValidPoint; p < pointCloud->end(); ++p)
    {
        if (std::isnan(p->x))
        {
            ++nans;
            continue;
        }

        tf::Vector3 point(p->x, p->y, p->z);
        point = transformation * point;
        point -= transformation.getOrigin();

#ifdef LOG_DETAILS
        xmin = std::min(xmin, (float)point.x());
        ymin = std::min(ymin, (float)point.y());
        zmin = std::min(zmin, (float)point.z());
        xmax = std::max(xmax, (float)point.x());
        ymax = std::max(ymax, (float)point.y());
        zmax = std::max(zmax, (float)point.z());
#endif

        //Parameters::Vec3Type pixel(point->x, point->y, point->z);
        Parameters::NumType measuredRange = point.length();
        point.normalize();
        Parameters::Vec3Type pixelDirection(point.x(), point.y(), point.z());
        Sensor sensor(origin, pixelDirection); // TODO check Parameters::sensorRange
        measurements.push_back(Measurement::voxel(sensor.ray(), measuredRange));
    }
#ifdef LOG_DETAILS
    ROS_INFO("PointCloud");
    ROS_INFO("\tDimensions:");
    ROS_INFO("\t\tMin: %f %f %f", xmin, ymin, zmin);
    ROS_INFO("\t\tMax: %f %f %f", xmax, ymax, zmax);
    ROS_INFO("\tNaN points: %d of %d (%.2f%%)", (int)nans, (int)pointCloud->size(), (float)(nans*100./pointCloud->size()));
#endif
    Observation observation(measurements);
    publishObservation(observation);
}

#endif

void Drone::run()
{
    _history.clear();
    ros::NodeHandle nodeHandle;
    message_filters::Subscriber<PointsMessage> pointsSub(
            nodeHandle,
            ROSTOPIC_POINTS,
            1);

    message_filters::Subscriber<TransformationMessage> transformationSub(
            nodeHandle,
            ROSTOPIC_TRANSFORMATION,
            1);

    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(100), pointsSub, transformationSub);
    sync.registerCallback(boost::bind(&Drone::handleMeasurements, this, _1, _2));

#ifndef PREPROCESSED_INPUT
    _tfPointCloudPub = nodeHandle.advertise<sensor_msgs::PointCloud2>("tf_points", 10);
    _tfDownsampledPointCloudPub = nodeHandle.advertise<sensor_msgs::PointCloud2>("tf_downsampled_points", 10);
    _tfDronePub = nodeHandle.advertise<geometry_msgs::TransformStamped>("tf_drone", 10);
#endif

    _stopRequested = false;
    while (!_stopRequested && ros::ok())
    {
        ros::spinOnce();
    }
}

void Drone::stop()
{
    _stopRequested = true;
}

bool queryBag(rosbag::ConnectionInfo const *connectionInfo)
{
    return (connectionInfo->topic == "tf_points" || connectionInfo->topic == "tf_drone");
}

void Drone::runBagFile(std::string filename)
{
    rosbag::Bag bag;
    bag.open(filename, rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back("tf_points");
    //topics.push_back("tf_drone");
    rosbag::View view(bag); //, rosbag::TopicQuery(topics));

    ROS_INFO("Reading bag messages....");
    ROS_INFO("View size: %d", (int)view.size());

//    rosbag::View view(bag, std::bind(queryBag, std::placeholders::_1));
    PointsMessageConstPtr points = nullptr;
    TransformationMessageConstPtr transformation = nullptr;
    int step = 0;
    for (rosbag::MessageInstance &msg : view)
    {
        ++step;
        if (msg.getTopic() == "/tf_points")
        {
            points = msg.instantiate<PointsMessage>();
        }
        else if (msg.getTopic() == "/tf_drone")
        {
            transformation = msg.instantiate<TransformationMessage>();
        }
        if (points != nullptr && transformation != nullptr)
        {
            handleMeasurements(points, transformation);
            points = nullptr;
            transformation = nullptr;
            if (step % 10 == 0)
                ROS_INFO("Updated for message %d.", step);
        }
    }

    bag.close();
}

void Drone::runCarmenFile(std::string filename, std::string messageName,
                          bool oldFormat, const unsigned int everyNth)
{
    std::fstream file(filename);
    if (!file)
    {
        ROS_ERROR("Error while reading CARMEN log file.");
        return;
    }

    _history.clear();
    std::string line;
    unsigned int flaserIdx = 0;
    unsigned int numReadings;
    float x, y, theta;
    const Eigen::Vector3f xAxis(1, 0, 0);
    const Eigen::Vector3f zAxis(0, 0, 1);
    while (std::getline(file, line))
    {
        std::istringstream ss(line);
        std::string type;
        ss >> type;
//        std::cout << type << std::endl;
//        if (type == "ODOM" && (everyNth == 0 || flaserIdx++ % everyNth == 0))
//        {
//            ss >> x >> y >> theta;
//            _history.add(TrajectoryPoint(Eigen::Vector3f(x, y, 0), theta));
//            std::cout << x << std::endl;
//        }
//        if (type == "TRUEPOS" && (everyNth == 0 || flaserIdx++ % everyNth == 0))
//        {
//            ss >> x >> y >> theta;
//            _history.add(TrajectoryPoint(Eigen::Vector3f(x, y, 0), theta));
//            std::cout << x << " " << y << " " << theta << std::endl;
//        }
        if (type == messageName)
        {
            std::vector<Measurement> measurements;
            if (oldFormat)
            {
                ss >> numReadings;
                std::vector<float> ranges(numReadings);
                for (unsigned int i = 0; i < numReadings; ++i)
                    ss >> ranges[i];
                ss >> x >> y >> theta;
//                std::cout << x << " " << y << " " << theta << std::endl;

                x = -x;
                y = -y;
                Parameters::Vec3Type origin(x, y, 0);

                theta += 0.5 * M_PI;
                _history.add(TrajectoryPoint(Eigen::Vector3f(x, y, 0), theta));

                if (everyNth == 0 || flaserIdx++ % everyNth == 0)
                {
                    for (unsigned int i = 0; i < numReadings; ++i)
                    {
                        if (ranges[i] > 20) // TODO read proper max range from CARMEN PARAMS
                            continue;
                        float angle = (float) (theta + M_PI * ((float) i) / ((float) numReadings));
//                    auto rotHorizontal = Eigen::AngleAxis<float>(angle, zAxis);
//                    Eigen::Vector3f orientation = rotHorizontal * xAxis;
//                    orientation.normalize();
                        Parameters::Vec3Type direction(std::cos(angle), std::sin(angle),
                                                       0.f); //orientation.x(), orientation.y(), orientation.z());
                        SensorRay sensor(origin, direction, ranges[i]);
                        measurements.push_back(Measurement::voxel(sensor, ranges[i]));
                    }
                }
            }
            else
            {
//                std::cout << "Found " << messageName << " message." << std::endl;
                int laserType, remissionMode;
                float startAngle, fov, angularResolution, maxRange, accuracy;
                ss >> laserType >> startAngle >> fov >> angularResolution
                   >> maxRange >> accuracy >> remissionMode >> numReadings;

                std::vector<float> ranges(numReadings);
                for (unsigned int i = 0; i < numReadings; ++i)
                    ss >> ranges[i];
                ss >> x >> x >> y >> theta; // TODO note how we skip one parameter
//                x -= 597;
                std::cout << x << " " << y << " " << theta << std::endl;

                Parameters::Vec3Type origin(x, y, 0);
                theta += 0.5*M_PI;
                Eigen::Vector3f orientation(std::cos(theta), std::sin(theta), 0.f);
                _history.add(TrajectoryPoint(Eigen::Vector3f(x, y, 0), orientation));
//                std::cout << "TP: " << (_history.end()-1)->position.x() << " " << (_history.end()-1)->position.y() << " " << theta << std::endl;
                if (everyNth == 0 || flaserIdx++ % everyNth == 0)
                {
                    for (unsigned int i = 0; i < numReadings; ++i)
                    {
                        float angle = startAngle + ((float) i) / (fov * numReadings);
                        auto rotHorizontal = Eigen::AngleAxis<float>(angle, zAxis);
                        Eigen::Vector3f orientation = rotHorizontal * xAxis;
                        orientation.normalize();
                        Parameters::Vec3Type direction(orientation.x(), orientation.y(), orientation.z());
                        SensorRay sensor(origin, direction, ranges[i]);
                        if (std::round(ranges[i]) >= std::round(maxRange))
                            measurements.push_back(Measurement::hole(sensor));
                        else
                            measurements.push_back(Measurement::voxel(sensor, ranges[i]));
                    }
                }
            }


//            ROS_INFO("Recorded %d measurements from CARMEN log line.", (int) measurements.size());

            //TODO reactivate
            Observation observation(measurements);
            publishObservation(observation);

#if LOG_DETAILS
            ROS_INFO("Published %i measurements from CARMEN logfile.", (int)numReadings);
#endif
        }
    }

    ROS_INFO("Recorded %i robot poses.", (int)_history.size());
}
