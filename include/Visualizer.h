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

    void render();

    void publishTrueMap(const Visualizable *visualizable);
    void publishTrueMap2dSlice(const Visualizable *visualizable, unsigned int z = 0);
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

    TrueMap *_lastTrueMap;
    BeliefMap *_lastBeliefMap;
};
