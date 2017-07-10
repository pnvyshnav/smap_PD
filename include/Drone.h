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


// Disable flag to transform and downsample incoming point clouds.
//#define PREPROCESSED_INPUT

#if INPUT_TYPE == INPUT_EUROC
    #ifdef PREPROCESSED_INPUT
        #define ROSTOPIC_POINTS "tf_downsampled_points"
        #define ROSTOPIC_TRANSFORMATION "tf_drone"
    #else
        #define ROSTOPIC_POINTS "points2"
        #define ROSTOPIC_TRANSFORMATION "vicon/firefly_sbx/firefly_sbx"
    #endif
#elif INPUT_TYPE == INPUT_SNAPDRAGON
    #define ROSTOPIC_POINTS "depth/image/raw"
    #define ROSTOPIC_TRANSFORMATION "vislam/pose"
#endif

class Drone : public Robot
{
public:
#if INPUT_TYPE == INPUT_EUROC
    typedef sensor_msgs::PointCloud2 PointsMessage;
    typedef geometry_msgs::TransformStamped TransformationMessage;
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
    void runBagFile(std::string filename);

    /**
     * Plays back a CARMEN log file with range sensor readings
     * in the form of:
     *      FLASER num_readings [range_readings] x y ...
     * @param filename CARMEN log file name.
     */
    void runCarmenFile(std::string filename);

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
