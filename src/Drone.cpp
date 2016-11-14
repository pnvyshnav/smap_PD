#include "../include/Drone.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <cmath>

#include "../include/Sensor.h"


typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::TransformStamped> SyncPolicy;

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


Drone::Drone() : _stopRequested(false)
{
}

void Drone::handleMeasurements(Drone::PointCloudMessage &pointsMsg, Drone::TransformationMessage &transformationMsg)
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
    pointCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(pointCloudFiltered);

    ROS_INFO("Point Cloud size reduced from %i to %i points.",
             (int)pointCloud->size(), (int)pointCloudFiltered->size());


    tf::StampedTransform stampedTransform;
    tf::transformStampedMsgToTF(*transformationMsg, stampedTransform);
    tf::Transform transformation(stampedTransform);

    transformation *= vicon * camera;

    //TODO broadcast tf and transformed PointCloud2 for debugging
    static tf::TransformBroadcaster br;
    auto time = ros::Time::now(); // uses simulation time from ros bag clock
    tf::StampedTransform tr2(transformation, time, "map", "tf_drone");
    br.sendTransform(tr2);
    geometry_msgs::TransformStamped tfMsg;
    tf::transformStampedTFToMsg(tr2, tfMsg);
    _tfDronePub.publish(tfMsg);

    sensor_msgs::PointCloud2 pointsTfMsg(*pointsMsg);
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
        auto sensorPtr = std::make_shared<Sensor>(sensor);
        measurements.push_back(Measurement::voxel(sensorPtr, measuredRange));
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

void Drone::run()
{
    ros::NodeHandle nodeHandle;
    message_filters::Subscriber<sensor_msgs::PointCloud2> pointsSub(
            nodeHandle,
#ifdef PREPROCESSED_INPUT
            "tf_downsampled_points",
#else
            "points2",
#endif
            1);

    message_filters::Subscriber<geometry_msgs::TransformStamped> transformationSub(
            nodeHandle,
#ifdef PREPROCESSED_INPUT
            "tf_drone",
#else
            "vicon/firefly_sbx/firefly_sbx",
#endif
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

