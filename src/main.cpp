#include <octomap/octomap.h>

#include <ros/ros.h>

#include "../include/BeliefVoxel.h"
#include "../include/BeliefMap.h"
#include "../include/FakeRobot.hpp"
#include "../include/Visualizer.h"
#include "../include/Drone.h"
#include "../include/LogOddsMap.h"
#include "../include/Statistics.h"
#include "../include/TrajectoryPlanner.h"

TrueMap trueMap = TrueMap::generate(123); // TODO leave out seed value
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

TrajectoryPlanner planner(trueMap, beliefMap, logOddsMap);


Statistics<> *stats;

void handleObservation(const Observation &observation)
{
    trueMap.publish();
    robot.publish();
    beliefMap.update(observation, trueMap);
    logOddsMap.update(observation, trueMap);

    stats->update(logOddsMap, beliefMap, robot);

//    beliefErrors.push_back(beliefMap.error(trueMap));
//    logOddsErrors.push_back(logOddsMap.error(trueMap));

#if defined(FAKE_2D) || defined(FAKE_3D)
    if (!ros::ok())
        robot.stop();
#endif
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "SMAP");
    ros::Time::init();
    stats = new Statistics<>(trueMap);
    //trueMap.writeBinary("simple_tree.bt");

    Visualizer *visualizer = new Visualizer;
    trueMap.subscribe(std::bind(&Visualizer::publishTrueMap, visualizer, std::placeholders::_1));
    beliefMap.subscribe(std::bind(&Visualizer::publishBeliefMapFull, visualizer, std::placeholders::_1));
    logOddsMap.subscribe(std::bind(&Visualizer::publishLogOddsMap, visualizer, std::placeholders::_1));
    robot.subscribe(std::bind(&Visualizer::publishFakeRobot, visualizer, std::placeholders::_1, &trueMap));
    planner.subscribe(std::bind(&Visualizer::publishTrajectoryPlanner, visualizer, std::placeholders::_1));

    robot.setReplanningHandler(std::bind(&TrajectoryPlanner::replan, planner,
                                         std::placeholders::_1,
                                         std::placeholders::_2,
                                         std::placeholders::_3));

    trueMap.publish();

#if defined(FAKE_2D)
    trueMap.subscribe(std::bind(&Visualizer::publishTrueMap2dSlice, visualizer, std::placeholders::_1, 0));
    robot.sensor().subscribe(std::bind(&Visualizer::publishStereoCameraSensor, visualizer, std::placeholders::_1));
    robot.registerObserver(&handleObservation);
    #if defined(PLANNER_2D_TEST)
        unsigned int splineId = 0;
        for (auto &trajectory : planner.generateTrajectories())
        {
            ROS_INFO("Evaluating spline %d...", (int)splineId);
            beliefMap.reset();
            logOddsMap.reset();
            robot.setTrajectory(trajectory);
            robot.run();
            planner.evaluate(trajectory, beliefMap, stats->stats());
            stats->saveToFile("trajeval/trajectory_" + std::to_string(splineId) + ".bag");
            stats->reset();
            ++splineId;
        }
    #else
        robot.run();
    #endif
#elif defined(FAKE_3D)
    trueMap.subscribe(std::bind(&Visualizer::publishTrueMap, visualizer, std::placeholders::_1));
    robot.sensor().subscribe(std::bind(&Visualizer::publishStereoCameraSensor, visualizer, std::placeholders::_1));
    robot.registerObserver(&handleObservation);
    for (unsigned int round = 0; round < 10; ++round)
    {
        beliefMap.reset();
        logOddsMap.reset();
        robot.run();
        stats->saveToFile("repeated_fake_3d/stats_" + std::to_string(round) + ".bag");
        stats->reset();
        ROS_INFO("Completed round %d", (int)round);
    }
#else
    Drone drone;
    drone.registerObserver(&handleObservation);
    drone.run();
#endif

    //stats->update(logOddsMap, beliefMap, robot);
    // TODO reactivate

#ifndef PLANNER_2D_TEST
    stats->saveToFile("/home/eric/catkin_ws/src/smap/stats/stats.bag");
#endif

    delete stats;

    visualizer->render();

    return EXIT_SUCCESS;
}
