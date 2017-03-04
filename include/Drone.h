#pragma once

#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/node_handle.h>
#include <tf/transform_datatypes.h>
#include <tf2_msgs/TFMessage.h>

#include "Robot.hpp"


// Disable flag to transform and downsample incoming point clouds.
//#define PREPROCESSED_INPUT

class Drone : public Robot
{
public:
    typedef const boost::shared_ptr<const sensor_msgs::PointCloud2> PointCloudMessage;
    typedef const boost::shared_ptr<const geometry_msgs::TransformStamped> TransformationMessage;

    Drone();

    void run();
    void stop();

    /**
     * Drone plays back bag file with given file name.
     * @param filename ROS bag file name.
     */
    void runOffline(std::string filename);

private:
    void handleMeasurements(PointCloudMessage &pointsMsg,
                            TransformationMessage &transformation);
    bool _stopRequested;

    ros::Publisher _tfPointCloudPub;
    ros::Publisher _tfDownsampledPointCloudPub;
    ros::Publisher _tfDronePub;

    static const tf::Transform vicon;
    static const tf::Transform camera;
};
