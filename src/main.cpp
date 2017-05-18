#include <octomap/octomap.h>

#include <ros/ros.h>

#include <ecl/time/stopwatch.hpp>

#include "../include/BeliefVoxel.h"
#include "../include/BeliefMap.h"
#include "../include/FakeRobot.hpp"
#include "../include/Visualizer.h"
#include "../include/Drone.h"
#include "../include/LogOddsMap.h"
#include "../include/Statistics.hpp"
#include "../include/PointCloud.h"

#ifdef REAL_3D
    TrueMap trueMap = TrueMap::generateFromPointCloud("~/catkin_ws/src/smap/dataset/V1_01_easy/data.ply");
#else
    TrueMap trueMap = TrueMap::generate(123); // use a fixed seed value
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


Statistics<> *stats;

ecl::StopWatch stopWatch;

int updated = 0;
void handleObservation(const Observation &observation)
{
#ifndef SLIM_STATS
    stats->registerMeasurements((int)observation.measurements().size());
    std::valarray<Parameters::NumType> rayLengths(observation.measurements().size());
    unsigned int i = 0;
    for (auto &measurement : observation.measurements())
    {
        rayLengths[i++] = measurement.value;
    }
    stats->registerRayStatistics(rayLengths.min(), rayLengths.max(), rayLengths.sum() / i);

    stopWatch.restart();
    beliefMap.update(observation, trueMap);
    stats->registerStepTimeBelief(stopWatch.elapsed());
    stopWatch.restart();
    logOddsMap.update(observation, trueMap);
    stats->registerStepTimeLogOdds(stopWatch.elapsed());
//
    stats->update(logOddsMap, beliefMap, robot);
#else
    beliefMap.update(observation, trueMap);
    logOddsMap.update(observation, trueMap);
#endif

#ifdef REAL_3D
    if (updated > 0 && updated % 25 == 0)
    {
#ifdef SLIM_STATS
        stats->update(logOddsMap, beliefMap, robot);
#endif

        // save stats continually
        stats->saveToFile("~/catkin_ws/src/smap/stats/stats_real3d_"
                          + std::to_string(updated) + ".bag");
#ifdef SLIM_STATS
        stats->reset();
#endif
    }
#endif
    ++updated;

#if defined(FAKE_2D) || defined(FAKE_3D)
    trueMap.publish();
    robot.publish();
    if (!ros::ok())
        robot.stop();
#endif
}

int main(int argc, char **argv)
{
//    PointCloud cloud;
//    cloud.loadPly("~/catkin_ws/src/smap/dataset/V1_01_easy/groundtruth_pcl.ply");
//    cloud.visualize();

    ros::init(argc, argv, "SMAP");
    ros::Time::init();

    stats = new Statistics<>(trueMap);

#ifdef PLANNER_2D_TEST
    TrajectoryPlanner planner(trueMap, beliefMap, logOddsMap);
#endif

#ifdef ENABLE_VISUALIZATION
    Visualizer *visualizer = new Visualizer;
    trueMap.subscribe(std::bind(&Visualizer::publishTrueMap, visualizer, std::placeholders::_1));
    for (int i = 0; i < 20; ++i)
        trueMap.publish();
#ifndef REAL_3D
    robot.subscribe(std::bind(&Visualizer::publishFakeRobot, visualizer, std::placeholders::_1, &trueMap));
#endif
//    beliefMap.subscribe(std::bind(&Visualizer::publishBeliefMap, visualizer, std::placeholders::_1));
//
//    //trueMap.subscribe(std::bind(&Visualizer::publishTrueMap2dSlice, visualizer, std::placeholders::_1, 0));
//    trueMap.subscribe(std::bind(&Visualizer::publishTrueMap, visualizer, std::placeholders::_1));
//
//    for (int j = 0; j < 20; ++j)
//        trueMap.publish();
//
//    visualizer->render();
//    return 0;

#ifdef PLANNER_2D_TEST
    planner.subscribe(std::bind(&Visualizer::publishTrajectoryPlanner, visualizer, std::placeholders::_1));
#endif
#ifdef FAKE_2D
    beliefMap.subscribe(std::bind(&Visualizer::publishBeliefMapFull, visualizer, std::placeholders::_1));
#else
    // todo reactivate
    beliefMap.subscribe(std::bind(&Visualizer::publishBeliefMapFull, visualizer, std::placeholders::_1));
    logOddsMap.subscribe(std::bind(&Visualizer::publishLogOddsMapFull, visualizer, std::placeholders::_1));
#endif
    robot.sensor().subscribe(std::bind(&Visualizer::publishStereoCameraSensor, visualizer, std::placeholders::_1));
#endif

    robot.registerObserver(&handleObservation);

#if defined(FAKE_2D)
    #if defined(PLANNER_2D_TEST)
        #if defined(REPLANNING)
            robot.setReplanningHandler(std::bind(&TrajectoryPlanner::replan, planner,
                                         std::placeholders::_1,
                                         std::placeholders::_2,
                                         std::placeholders::_3));
            //robot.setTrajectory(planner.replan(Point(0.05, -0.95), Point(-0.95, 0.05), 0.0));
            robot.setTrajectory(TrajectoryPlanner::generateInitialDirectTrajectory(Point(0.05, -0.95), Point(-0.95, 0.05)));
            robot.run();
            stats->saveToFile("replanning/replanning.bag");
        #else
            unsigned int splineId = 0;
            for (auto &trajectory : planner.generateTrajectories())
            {
                ROS_INFO("Evaluating spline %d...", (int)splineId);
                beliefMap.reset();
                logOddsMap.reset();
                robot.setTrajectory(trajectory);
                robot.run();
                planner.evaluate(trajectory, beliefMap, stats->stats());
        #ifdef ONLY_HANDCRAFTED_TRAJECTORIES
                stats->saveToFile("handcrafted_trajeval/trajectory_" + std::to_string(splineId) + ".bag");
        #else
                stats->saveToFile("time trajeval mlCause one pixel singlePoint/trajectory_" + std::to_string(splineId) + ".bag");
        #endif
                stats->reset();
                ++splineId;
            }
        #endif
    #else
        robot.run();
    #endif
#elif defined(FAKE_3D)
    #ifdef ENABLE_VISUALIZATION
        trueMap.subscribe(std::bind(&Visualizer::publishTrueMap, visualizer, std::placeholders::_1));
        robot.sensor().subscribe(std::bind(&Visualizer::publishStereoCameraSensor, visualizer, std::placeholders::_1));
    #endif

    #if defined(REPEATED_RUNS)
        for (unsigned int round = 0; round < 1; ++round)
        {
            beliefMap.reset();
            logOddsMap.reset();
            robot.run();
            //stats->saveToFile("different_maps/stats_" + std::to_string(round) + ".bag");
            stats->saveToFile("repeated_fullmap_noise0.01_n/stats_" + std::to_string(round) + ".bag");
            stats->reset();
            ROS_INFO("Completed round %d", (int)round);
            trueMap.shuffle();
        }
    #elif defined(ISM_RUNS)
        std::vector<double> increments = {0.05, 0.2, 0.4};
        std::vector<double> rampSizes = {0.05, 0.1, 0.3};
        std::vector<double> topSizes = {0.05, 0.1, 0.3};
        unsigned int round = 0;
        for (auto increment : increments)
        {
            for (auto rampSize : rampSizes)
            {
                for (auto topSize : topSizes)
                {
                    ROS_INFO("Running with ISM parameters: increment=%f rampSize=%f topSize=%f",
                             increment, rampSize, topSize);
                    LogOddsMap::parameters.increment = increment;
                    LogOddsMap::parameters.rampSize = rampSize;
                    LogOddsMap::parameters.topSize = topSize;
                    beliefMap.reset();
                    logOddsMap.reset();
                    robot.run();
                    stats->saveToFile("ism_runs_fewsteps_noise0.05/stats_" + std::to_string(round++) + ".bag");
                    stats->reset();
                }
            }
        }
    #else
        robot.run();
    #endif
#else
    Drone drone;
    drone.registerObserver(&handleObservation);
//    drone.run();
    drone.runOffline("~/catkin_ws/src/smap/dataset/V1_01_easy/droneflight.bag");
#endif

#ifndef PLANNER_2D_TEST
//    visualizer->publishBeliefMapFull(&beliefMap);
//    visualizer->publishLogOddsMapFull(&logOddsMap);
//    visualizer->publishTrueMap(&trueMap);
    stats->saveToFile("~/catkin_ws/src/smap/stats/stats.bag");
#endif

    delete stats;

#ifdef ENABLE_VISUALIZATION
    visualizer->render();
#endif

    return EXIT_SUCCESS;
}
