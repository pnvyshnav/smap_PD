#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sensor_msgs/LaserScan.h>

#include "../include/Parameters.h"
#include "../include/Observation.hpp"
#include "../include/Visualizer.h"


// if no CLI arg given, open this bag file
#define DEFAULT_BAG_FILENAME "/home/wal/catkin_ws/src/smap/lidar_pd/lidar_vicon_2018-02-13-17-06-14.bag"

// save belief map at this location
#define SAVE_MAP_FILENAME "/home/wal/catkin_ws/src/smap/BeliefMap.bin"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "CRM");

    rosbag::Bag bag;
    if (argc > 1)
    {
        std::cout << "Reading bag file from " << argv[1] << "." << std::endl;
        bag.open(argv[1], rosbag::bagmode::Read);
    }
    else
    {
        bag.open(DEFAULT_BAG_FILENAME, rosbag::bagmode::Read);
    }

    std::vector<std::string> topics;
    topics.emplace_back(std::string("/scan"));
    topics.emplace_back(std::string("/vicon/hokuyo_lidar/hokuyo_lidar"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    // make sure to set these parameters before constructing any objects from this project
    // I guessed these limits (the smaller the better the performance as less voxels need to be computed)
    Parameters::xMin = -4.7;
    Parameters::yMin = -4.7;
    Parameters::zMin = -0.3;
    Parameters::xMax = 3.8;
    Parameters::yMax = 3.7;
    Parameters::zMax = 5.2;
    Parameters::voxelSize = 0.125;
    Parameters::sensorRange = 5;
    Parameters::sensorNoiseStd = 0.01;

    BeliefMap beliefMap;
    TrueMap trueMap;

    auto *visualizer = new Visualizer;
    // this will only publish the updated voxels at every step on ROS topic /belief_map
    beliefMap.subscribe(std::bind(&Visualizer::publishBeliefMap, visualizer, std::placeholders::_1, 0.55));

    Observation allObs;
    tf::Quaternion rotation;
    Parameters::Vec3Type position;
    unsigned int msgCounter = 0;
    for (auto &msg : view)
    {
        sensor_msgs::LaserScan::ConstPtr ls_msg = msg.instantiate<sensor_msgs::LaserScan>();
        geometry_msgs::TransformStamped::ConstPtr tf_msg = msg.instantiate<geometry_msgs::TransformStamped>();
        if (tf_msg != nullptr)
        {
            // load transformation
            position = Parameters::Vec3Type((float) tf_msg->transform.translation.x,
                                            (float) tf_msg->transform.translation.y,
                                            (float) tf_msg->transform.translation.z);
            tf::quaternionMsgToTF(tf_msg->transform.rotation, rotation);
        }
        if (ls_msg != nullptr)
        {
            // build observation from laser scan
            Observation observation;

            unsigned int i = 0;
            for (auto range : ls_msg->ranges)
            {
                double theta = ls_msg->angle_min + i * ls_msg->angle_increment;
                auto pixel = tf::Vector3(std::cos(theta), std::sin(theta), 0);
                tf::Vector3 tf_orientation = tf::quatRotate(rotation, pixel);
                Parameters::Vec3Type orientation((float) tf_orientation.x(),
                                                 (float) tf_orientation.y(),
                                                 (float) tf_orientation.z());
                SensorRay ray(position, orientation, range);
                if (range <= 0. || range > 10. || std::isnan(range) || std::isinf(range))
                    observation.append(Measurement::hole(ray));
                else
                    observation.append(Measurement::voxel(ray, range));
                ++i;
            }
            allObs.append(observation);

            if (msgCounter % 50 == 0)
            {
                // visualize last 50 scans on /observation
                for (int j = 0; j < 1; ++j)
                {
                    visualizer->publishObservation(&allObs, false, true, 20);
                    visualizer->sleep(10);
                }
                allObs.clear();
            }

            // update CRM
            beliefMap.update(observation, trueMap);
            msgCounter++;
            if (msgCounter > 10)
                break;
        }
    }

    bag.close();

    // save as binary file
    // TODO test BeliefMap::load
    beliefMap.save(SAVE_MAP_FILENAME);

    beliefMap.loadFromFile(SAVE_MAP_FILENAME);

    return EXIT_SUCCESS;
}
