#include "../include/Drone.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <cmath>

#include "../include/Sensor.h"

typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::PointCloud2, geometry_msgs::TransformStamped> SyncPolicy;

Drone::Drone() : _stopRequested(false)
{
}

void Drone::handleMeasurements(Drone::PointCloudMessage &pointsMsg, Drone::TransformationMessage &transformationMsg)
{
    // convert messages to usable datatypes
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*pointsMsg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *pointCloud);

    tf::StampedTransform transformation;
    tf::transformStampedMsgToTF(*transformationMsg, transformation);

    Parameters::Vec3Type origin((float) transformation.getOrigin().x(),
                       (float) transformation.getOrigin().y(),
                       (float) transformation.getOrigin().z());
    tf::Vector3 tfdirection = tf::Vector3(0, 0, 1).rotate(
            transformation.getRotation().getAxis(),
            transformation.getRotation().getAngle());
    Parameters::Vec3Type direction(tfdirection.x(), tfdirection.y(), tfdirection.z());

    ROS_INFO("Sensor Position:  %.3f %.3f %.3f", origin.x(), origin.y(), origin.z());
    ROS_INFO("Sensor Direction: %.3f %.3f %.3f", direction.x(), direction.y(), direction.z());

    if (std::isnan(origin.x()))
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

    float xmin = firstValidPoint->x;
    float ymin = firstValidPoint->y;
    float zmin = firstValidPoint->z;
    float xmax = firstValidPoint->x;
    float ymax = firstValidPoint->y;
    float zmax = firstValidPoint->z;

    unsigned int nans = 0;
    std::vector<Measurement> measurements;
    for (auto &point = firstValidPoint; point < pointCloud->end(); point+=1000)
    {
        if (std::isnan(point->x))
        {
            ++nans;
            continue;
        }

        point->x -= origin.x();
        point->y -= origin.y();
        point->z -= origin.z();

        xmin = std::min(xmin, point->x);
        ymin = std::min(ymin, point->y);
        zmin = std::min(zmin, point->z);
        xmax = std::max(xmax, point->x);
        ymax = std::max(ymax, point->y);
        zmax = std::max(zmax, point->z);

        Parameters::Vec3Type pixel(point->x, point->y, point->z);
        Parameters::NumType range = pixel.norm();
        Parameters::Vec3Type pixelDirection = pixel.normalize() - direction;
        pixelDirection.normalize();
        Sensor sensor(Parameters::Vec3Type(origin), Parameters::Vec3Type(pixelDirection), range);
        auto sensorPtr = std::make_shared<Sensor>(sensor);
        measurements.push_back(Measurement::voxel(sensorPtr, range));
    }

    ROS_INFO("PointCloud");
    ROS_INFO("\tDimensions:");
    ROS_INFO("\t\tMin: %f %f %f", xmin, ymin, zmin);
    ROS_INFO("\t\tMax: %f %f %f", xmax, ymax, zmax);
    ROS_INFO("\tNaN points: %d of %d (%.2f%%)", (int)nans, (int)pointCloud->size(), (float)(nans*100./pointCloud->size()));

    Observation observation(measurements);
    publishObservation(observation);
}

void Drone::run()
{
    ros::NodeHandle nodeHandle;
    message_filters::Subscriber<sensor_msgs::PointCloud2> pointsSub(nodeHandle, "points2", 1);
    message_filters::Subscriber<geometry_msgs::TransformStamped> transformationSub(nodeHandle, "vicon/firefly_sbx/firefly_sbx", 1);
    //message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, geometry_msgs::TransformStamped> sync(pointsSub, transformationSub, 10);
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(100), pointsSub, transformationSub);
    sync.registerCallback(boost::bind(&Drone::handleMeasurements, this, _1, _2));

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

