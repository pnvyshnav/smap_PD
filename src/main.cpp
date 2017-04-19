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
    TrueMap trueMap = TrueMap::generateRandomCorridor(); // use a fixed seed value
#endif

MapType map;
BeliefMap beliefMap;
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
        map);

ecl::StopWatch stopWatch;

Visualizer *visualizer;

int updated = 0;
void handleObservation(const Observation &observation)
{
#ifndef SLIM_STATS
    stopWatch.restart();
    map.update(observation, trueMap);
    stopWatch.restart();
#else
    map.update(observation, trueMap);
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

    robot.registerObserver(&handleObservation);
    robot.setYaw(M_PI);

    visualizer = new Visualizer(&trueMap, &map, &robot);

//    robot.setPosition(Parameters::Vec3Type(0.35f, -0.85f, 0));
    robot.run();

    visualizer->render();

    return EXIT_SUCCESS;
}
