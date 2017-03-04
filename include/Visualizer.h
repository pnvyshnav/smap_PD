#pragma once

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

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
    void publishLogOddsMapFull(const Observable *visualizable);
    void publishBeliefMap(const Observable *visualizable);
    void publishBeliefMapFull(const Observable *visualizable);
    void publishSensor(const Observable *visualizable);
    void publishStereoCameraSensor(const Observable *visualizable);
    void publishRay(TrueMap &trueMap, Sensor &sensor);
    void publishFakeRobot(const Observable *visualizable, const TrueMap *trueMap);
    void publishTrajectoryPlanner(const Observable *visualizable);

    static const int PaintRate = 2000;

private:
    int _counter;
    ros::NodeHandle *nodeHandle;
    ros::Publisher trueMapPublisher;
    ros::Publisher trueMap2dPublisher;
    ros::Publisher logOddsMapPublisher;
    ros::Publisher beliefMapPublisher;
    ros::Publisher rayVoxelPublisher;
    ros::Publisher sensorPublisher;
    ros::Publisher stereoCameraSensorPublisher;
    ros::Publisher splinePublisher;
    ros::Publisher evaluationPublisher;
    ros::Publisher trajectoryVoxelsPublisher;
    ros::Publisher finalTrajectoryPublisher;
};
