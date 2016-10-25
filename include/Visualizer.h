#pragma once

#include <ros/ros.h>

#include "Visualizable.hpp"
#include "TrueMap.h"
#include "Sensor.h"


class Visualizer
{
public:
    Visualizer(int argc, char **argv);
    ~Visualizer();

    static void render();

    void publishTrueMap(const Visualizable *visualizable);
    void publishBeliefMap(const Visualizable *visualizable);
    void publishSensor(const Visualizable *visualizable);
    void publishRay(TrueMap &trueMap, Sensor &sensor);

private:
    ros::NodeHandle *nodeHandle;
    ros::Publisher trueMapPublisher;
    ros::Publisher trueMap2dPublisher;
    ros::Publisher beliefMapPublisher;
    ros::Publisher rayVoxelPublisher;
    ros::Publisher sensorPublisher;
};
