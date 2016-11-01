#pragma once

#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/node_handle.h>
#include <tf/transform_datatypes.h>

#include "Robot.hpp"

class Drone : public Robot
{
public:
    typedef const boost::shared_ptr<const sensor_msgs::PointCloud2> PointCloudMessage;
    typedef const boost::shared_ptr<const geometry_msgs::TransformStamped> TransformationMessage;

    Drone();

    void run();
    void stop();

private:
    void handleMeasurements(PointCloudMessage &pointsMsg,
                            TransformationMessage &transformation);
    bool _stopRequested;

    ros::Publisher _tfPointCloudPub;

    static const tf::Transform vicon;
    static const tf::Transform camera;
};
