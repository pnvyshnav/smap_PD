#pragma once

#include <ros/ros.h>

#include "Observable.hpp"
#include "TrueMap.h"
#include "Sensor.h"


class Visualizer
{
public:
    Visualizer();
    virtual ~Visualizer();

    void render();

    void publishTrueMap(const Observable *visualizable);
    void publishTrueMap2dSlice(const Observable *visualizable, unsigned int z = 0);
    void publishLogOddsMap(const Observable *visualizable);
    void publishBeliefMap(const Observable *visualizable);
    void publishBeliefMapFull(const Observable *visualizable);
    void publishSensor(const Observable *visualizable);
    void publishStereoCameraSensor(const Observable *visualizable);
    void publishRay(TrueMap &trueMap, Sensor &sensor);

    static const int PaintRate = 200;

private:
    ros::NodeHandle *nodeHandle;
    ros::Publisher trueMapPublisher;
    ros::Publisher trueMap2dPublisher;
    ros::Publisher logOddsMapPublisher;
    ros::Publisher beliefMapPublisher;
    ros::Publisher rayVoxelPublisher;
    ros::Publisher sensorPublisher;
    ros::Publisher stereoCameraSensorPublisher;

    TrueMap *_lastTrueMap;
    BeliefMap *_lastBeliefMap;
};
