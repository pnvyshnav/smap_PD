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
    void sleep(int milliseconds = PaintRate);

    void publishTrueMap(const Observable *visualizable);
    void publishTrueMap2dSlice(const Observable *visualizable, unsigned int z = 0);
    void publishLogOddsMap(const Observable *visualizable);
    void publishLogOddsMapFull(const Observable *visualizable, bool visualizeStd = false);
    void publishBeliefMap(const Observable *visualizable, double threshold = 0.52);
    void publishBeliefMapFull(const Observable *visualizable, bool visualizeStd = false);
    void publishSensor(const Observable *visualizable);
    void publishStereoCameraSensor(const Observable *visualizable);
    void publishRay(TrueMap &trueMap, Sensor &sensor);
    void publishFakeRobot(const Observable *visualizable, const TrueMap *trueMap);
    void publishGaussianProcessMapFull(const Observable *visualizable, bool visualizeStd = true);
    void publishTrajectory(const Observable *visualizable);
    void publishObservation(const Observable *observation, bool visualizeRays = true,
                            bool removeOld = true, int sparsificationFactor = 1);

    void publishBeliefInconsistencyMapFull(const Observable *visualizable, TrueMap &trueMap, double k);
    void publishLogOddsInconsistencyMapFull(const Observable *visualizable, TrueMap &trueMap, double k);
    void publishGaussianProcessInconsistencyMapFull(const Observable *visualizable, TrueMap &trueMap, double k);

    static const int PaintRate = 200;


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
    ros::Publisher trajectoryVoxelsPublisher;
    ros::Publisher finalTrajectoryPublisher;
    ros::Publisher gaussianProcessMapPublisher;
    ros::Publisher trajectoryPublisher;
    ros::Publisher observationPublisher;

    static double hue2rgb(double arg1, double arg2, double h);
    static void hsl2rgb(double h, double s, double l, double &r, double &g, double &b);
};
