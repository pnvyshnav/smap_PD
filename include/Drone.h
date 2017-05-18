#pragma once

#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/node_handle.h>
#include <tf/transform_datatypes.h>
#include <tf2_msgs/TFMessage.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "Robot.hpp"

#define INPUT_EUROC 1       // EuRoC MAV dataset
#define INPUT_SNAPDRAGON 2  // Snapdragon Flight

#define INPUT_TYPE INPUT_SNAPDRAGON


// Disable flag to transform and downsample incoming point clouds.
//#define PREPROCESSED_INPUT

class Drone : public Robot
{
public:
#if INPUT_TYPE == INPUT_EUROC
    typedef const sensor_msgs::PointCloud2 PointsMessage;
    typedef const geometry_msgs::TransformStamped TransformationMessage;
#elif INPUT_TYPE == INPUT_SNAPDRAGON
    typedef sensor_msgs::Image PointsMessage;
    typedef geometry_msgs::PoseStamped TransformationMessage;
#endif
    typedef PointsMessage::ConstPtr PointsMessageConstPtr;
    typedef TransformationMessage::ConstPtr TransformationMessageConstPtr;

    typedef message_filters::sync_policies::ApproximateTime<PointsMessage, TransformationMessage> SyncPolicy;

    Drone();

    void run();
    void stop();

    /**
     * Drone plays back bag file with given file name.
     * @param filename ROS bag file name.
     */
    void runOffline(std::string filename);

private:
#if INPUT_TYPE == INPUT_EUROC
    void handleMeasurements(PointsMessageConstPtr pointsMsg,
                            TransformationMessageConstPtr transformation);
#elif INPUT_TYPE == INPUT_SNAPDRAGON
    void handleMeasurements(PointsMessageConstPtr depthImage, TransformationMessageConstPtr pose);
#endif

    bool _stopRequested;

    ros::Publisher _tfPointCloudPub;
    ros::Publisher _tfDownsampledPointCloudPub;
    ros::Publisher _tfDronePub;

#if INPUT_TYPE == INPUT_EUROC
    static const tf::Transform vicon;
    static const tf::Transform camera;
#endif
};
