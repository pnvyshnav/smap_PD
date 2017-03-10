#include <octomap/octomap.h>

#include <ros/ros.h>
#include <Eigen/Dense>

#include <ecl/time/stopwatch.hpp>
#include <GL/freeglut.h>

#include "../include/BeliefVoxel.h"
#include "../include/BeliefMap.h"
#include "../include/FakeRobot.hpp"
#include "../include/Visualizer.h"
#include "../include/Drone.h"
#include "../include/LogOddsMap.h"
#include "../include/PointCloud.h"

#ifdef REAL_3D
    TrueMap trueMap = TrueMap::generateFromPointCloud("/home/eric/catkin_ws/src/smap/dataset/V1_01_easy/data.ply");
#else
//    TrueMap trueMap = TrueMap::generate(123); // use a fixed seed value
    TrueMap trueMap = TrueMap::generateCorridor(); // use a fixed seed value
#endif

BeliefMap beliefMap;
LogOddsMap logOddsMap;
FakeRobot<> robot(
        Parameters::Vec3Type(Parameters::xCenter,
                             Parameters::yCenter,
                             Parameters::zCenter),
#if defined(FAKE_2D)
        Parameters::Vec3Type(1, 0, 0),
#else
        Parameters::Vec3Type(0, 1, 0),
#endif
        trueMap,
        beliefMap);

ecl::StopWatch stopWatch;

Visualizer *visualizer;

int updated = 0;
void handleObservation(const Observation &observation)
{
#ifndef SLIM_STATS
    stopWatch.restart();
    beliefMap.update(observation, trueMap);
    stopWatch.restart();
    logOddsMap.update(observation, trueMap);
#else
    beliefMap.update(observation, trueMap);
    logOddsMap.update(observation, trueMap);
#endif

    ++updated;

    visualizer->update();
}

int main(int argc, char **argv)
{
    glutInit(&argc, argv);

    ros::init(argc, argv, "SMAP");
    ros::Time::init();

    visualizer = new Visualizer(&trueMap, &beliefMap, &robot);

    robot.setPosition(Parameters::Vec3Type(0.35, -0.95, 0));
    robot.setYaw(M_PI / 2.);
    robot.registerObserver(&handleObservation);
    robot.run();

    visualizer->render();

    return EXIT_SUCCESS;
}
