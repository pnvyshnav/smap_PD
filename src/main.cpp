#include <octomap/octomap.h>

#include <ros/ros.h>

#include <ecl/time/stopwatch.hpp>

#include "../include/BeliefVoxel.h"
#include "../include/BeliefMap.h"
#include "../include/LogOddsMap.h"
#include "../include/GaussianProcessMap.h"
#include "../include/FakeRobot.hpp"
#include "../include/Visualizer.h"
#include "../include/Drone.h"
#include "../include/Statistics.hpp"
#include "../include/PointCloud.h"
#include "../include/Trajectory.hpp"

std::string homedir = getenv("HOME");

#ifdef REAL_3D
    TrueMap trueMap = TrueMap::generateFromPointCloud(homedir + "/catkin_ws/src/smap/dataset/V1_01_easy/data.ply");
#elif defined(REAL_2D)
//    std::string carmenFile = homedir + "/catkin_ws/src/smap/dataset/fr_campus/fr-campus.carmen.log";
//    std::string carmenFile = homedir + "/catkin_ws/src/smap/dataset/mit-csail-3rd-floor-2005-12-17-run4.flaser.log";
    std::string carmenFile = homedir + "/catkin_ws/src/smap/dataset/albertB.img.sm.log";
    TrueMap trueMap = TrueMap::generateFromCarmen(carmenFile, "FLASER", true, 1);//, "ROBOTLASER1", false);
#else
    TrueMap trueMap = TrueMap::generateCorridor(); //TrueMap::generate(123); // use a fixed seed value
#endif

BeliefMap beliefMap;
LogOddsMap logOddsMap;
GaussianProcessMap gaussianProcessMap;
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

Observation allObservations;

int updated = 0;
void handleObservation(const Observation &observation)
{
    allObservations.append(observation);
#ifdef GP_RUNS
    return;
#endif
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

//    stopWatch.restart();
//    gaussianProcessMap.update(observation);
//    stats->registerStepTimeGP(stopWatch.elapsed());

    stats->update(logOddsMap, beliefMap, gaussianProcessMap, robot);
#else
    beliefMap.update(observation, trueMap);
    logOddsMap.update(observation, trueMap);
    gaussianProcessMap.update(observation);
#endif

#ifdef REAL_3D
    if (updated > 0 && updated % 25 == 0)
    {
#ifdef SLIM_STATS
        stats->update(logOddsMap, beliefMap, robot);
#endif

        // save stats continually
        stats->saveToFile(homedir + "/catkin_ws/src/smap/stats/stats_real3d_"
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
    gaussianProcessMap.publish();
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
    trueMap.subscribe(std::bind(&Visualizer::publishTrueMap2dSlice, visualizer, std::placeholders::_1, 0));
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
    double k_inconsistency = 1;
//    beliefMap.subscribe(std::bind(&Visualizer::publishBeliefMapFull,
//                                  visualizer, std::placeholders::_1,
//                                  true));
    beliefMap.subscribe(std::bind(&Visualizer::publishBeliefInconsistencyMapFull,
                                  visualizer, std::placeholders::_1,
                                  trueMap,
                                  k_inconsistency));
//    logOddsMap.subscribe(std::bind(&Visualizer::publishLogOddsMapFull,
//                                   visualizer, std::placeholders::_1,
//                                   true));
    logOddsMap.subscribe(std::bind(&Visualizer::publishLogOddsInconsistencyMapFull,
                                  visualizer, std::placeholders::_1,
                                  trueMap,
                                  k_inconsistency));
//    gaussianProcessMap.subscribe(std::bind(&Visualizer::publishGaussianProcessMapFull,
//                                           visualizer, std::placeholders::_1,
//                                           true));
    gaussianProcessMap.subscribe(std::bind(&Visualizer::publishGaussianProcessInconsistencyMapFull,
                                           visualizer, std::placeholders::_1,
                                           trueMap,
                                           k_inconsistency));
#else
    // todo reactivate
//    beliefMap.subscribe(std::bind(&Visualizer::publishBeliefMapFull, visualizer, std::placeholders::_1));
//    logOddsMap.subscribe(std::bind(&Visualizer::publishLogOddsMapFull, visualizer, std::placeholders::_1));
#endif
    robot.sensor().subscribe(std::bind(&Visualizer::publishStereoCameraSensor, visualizer, std::placeholders::_1));
#endif

    robot.registerObserver(&handleObservation);


    Trajectory trajectory = {
            TrajectoryPoint(Eigen::Vector3f(-0.92f, -0.25f, 0.f), (float)(   0.f * M_PI / 180.f)),
            TrajectoryPoint(Eigen::Vector3f(-0.77f, -0.25f, 0.f), (float)(   0.f * M_PI / 180.f)),
            TrajectoryPoint(Eigen::Vector3f(-0.61f, -0.25f, 0.f), (float)(   0.f * M_PI / 180.f)),
            TrajectoryPoint(Eigen::Vector3f(-0.46f, -0.25f, 0.f), (float)(   0.f * M_PI / 180.f)),
            TrajectoryPoint(Eigen::Vector3f(-0.30f, -0.25f, 0.f), (float)(  15.f * M_PI / 180.f)),
            TrajectoryPoint(Eigen::Vector3f(-0.16f, -0.21f, 0.f), (float)(  60.f * M_PI / 180.f)),
            TrajectoryPoint(Eigen::Vector3f(-0.08f, -0.07f, 0.f), (float)(  80.f * M_PI / 180.f)),
            TrajectoryPoint(Eigen::Vector3f(-0.05f, 0.09f, 0.f),  (float)(  90.f * M_PI / 180.f)),
            TrajectoryPoint(Eigen::Vector3f(-0.05f, 0.24f, 0.f),  (float)(  80.f * M_PI / 180.f)),
            TrajectoryPoint(Eigen::Vector3f(-0.03f, 0.39f, 0.f),  (float)(  85.f * M_PI / 180.f)),
            TrajectoryPoint(Eigen::Vector3f(-0.01f, 0.56f, 0.f),  (float)(  30.f * M_PI / 180.f)),
            TrajectoryPoint(Eigen::Vector3f(0.13f, 0.63f, 0.f),   (float)( -10.f * M_PI / 180.f)),
            TrajectoryPoint(Eigen::Vector3f(0.30f, 0.61f, 0.f),   (float)( -10.f * M_PI / 180.f)),
            TrajectoryPoint(Eigen::Vector3f(0.45f, 0.58f, 0.f),   (float)(   5.f * M_PI / 180.f)),
            TrajectoryPoint(Eigen::Vector3f(0.62f, 0.58f, 0.f),   (float)( -25.f * M_PI / 180.f)),
            TrajectoryPoint(Eigen::Vector3f(0.74f, 0.51f, 0.f),   (float)( -70.f * M_PI / 180.f)),
            TrajectoryPoint(Eigen::Vector3f(0.77f, 0.36f, 0.f),   (float)( -85.f * M_PI / 180.f)),
            TrajectoryPoint(Eigen::Vector3f(0.77f, 0.20f, 0.f),   (float)( -90.f * M_PI / 180.f)),
            TrajectoryPoint(Eigen::Vector3f(0.77f, 0.05f, 0.f),   (float)(-100.f * M_PI / 180.f)),
            TrajectoryPoint(Eigen::Vector3f(0.74f, -0.11f, 0.f),  (float)(-140.f * M_PI / 180.f)),
            TrajectoryPoint(Eigen::Vector3f(0.61f, -0.17f, 0.f),  (float)(-170.f * M_PI / 180.f)),
            TrajectoryPoint(Eigen::Vector3f(0.46f, -0.17f, 0.f),  (float)(-175.f * M_PI / 180.f)),
            TrajectoryPoint(Eigen::Vector3f(0.30f, -0.17f, 0.f),  (float)(-180.f * M_PI / 180.f))
    };

#if defined(ISM_RUNS)
    std::vector<double> increments = {0.05, 0.25, 0.4};
    std::vector<double> rampSizes = {0.025, 0.05, 0.1, 0.3};
    std::vector<double> topSizes = {0.025, 0.05, 0.1, 0.3};
    unsigned int round = 0;
#if defined(REAL_2D)
    Drone drone;
    drone.registerObserver(&handleObservation);
    visualizer->publishTrueMap2dSlice(&trueMap, 0);
    std::cout << "Running CARMEN file: " << carmenFile << std::endl;
#endif
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
#if defined(REAL_2D)
                drone.runCarmenFile(carmenFile, "FLASER", true, 10);//, "ROBOTLASER1", false);
#else
                robot.run(trajectory);
#endif

                stats->saveToFile(homedir + "/catkin_ws/src/smap/stats/ism_albert_runs/stats_" + std::to_string(round++) + ".bag");
                stats->reset();
            }
        }
    }
#elif defined(GP_RUNS)
    // obtain observations
    robot.run(trajectory);

    std::vector<double> p1s = {-2, -2.5, -3};
    std::vector<double> p2s = {-2.5, -3.5, -4.5};
    std::vector<double> p3s = {-0.5, -1, -1.5};
    unsigned int round = 0;
    for (auto p1 : p1s)
    {
        for (auto p2 : p2s)
        {
            for (auto p3 : p3s)
            {
                gaussianProcessMap.reset();
                ROS_INFO("Running with GP parameters: p1=%f p2=%f p3=%f",
                         p1, p2, p3);
                GaussianProcessMap::parameters.parameter1 = p1;
                GaussianProcessMap::parameters.parameter2 = p2;
                GaussianProcessMap::parameters.parameter3 = p3;
                gaussianProcessMap.updateParameters();
                gaussianProcessMap.update(allObservations);

                stats->update(logOddsMap, beliefMap, gaussianProcessMap, robot);

                stats->saveToFile(homedir + "/catkin_ws/src/smap/stats/gp_runs/stats_" + std::to_string(round++) + ".bag");
                stats->reset();
            }
        }
    }
    return EXIT_SUCCESS;

#elif defined(FAKE_2D)
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

        #ifdef ENABLE_VISUALIZATION
            visualizer->publishTrajectory(&trajectory);
        #endif

        robot.run(trajectory);
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
    #else
        robot.run();
    #endif
#elif defined(REAL_2D)
    trueMap.subscribe(std::bind(&Visualizer::publishTrueMap2dSlice, visualizer, std::placeholders::_1, 0));
    Drone drone;
    drone.registerObserver(&handleObservation);
    visualizer->publishTrueMap2dSlice(&trueMap, 0);
    std::cout << "Running CARMEN file: " << carmenFile << std::endl;
    drone.runCarmenFile(carmenFile, "FLASER", true, 10);//, "ROBOTLASER1", false);
#else
    Drone drone;
    drone.registerObserver(&handleObservation);
//    drone.run();
    drone.runOffline(homedir + "/catkin_ws/src/smap/dataset/V1_01_easy/droneflight.bag");
#endif

    // TODO compute Hilbert map using all measurements
//    gaussianProcessMap.update(allObservations);
//    gaussianProcessMap.publish();
//    stats->update(logOddsMap, beliefMap, gaussianProcessMap, robot);

    std::stringstream ss;
    ss << Parameters::sensorNoiseStd;
    stats->saveToFile(homedir + "/catkin_ws/src/smap/stats/stats_carmen_std_" + ss.str() + ".bag");
#ifdef ENABLE_VISUALIZATION
#if defined(REAL_2D) || defined(REAL_3D)
    trajectory = drone.poseHistory();
#endif
    std::cout << "Trajectory bounding box: " << trajectory.boundingBox().str() << std::endl;
    for (int i = 0; i < 1; i++)
    {
        visualizer->publishObservation(&allObservations);
        visualizer->publishTrajectory(&trajectory);
        visualizer->publishTrueMap2dSlice(&trueMap, 0);
        visualizer->sleep(1);
        visualizer->publishBeliefMapFull(&beliefMap);
        visualizer->publishLogOddsMapFull(&logOddsMap);
    }
#endif
    std::cout << allObservations.measurements().size()
              << " measurements were taken in total." << std::endl;

#ifndef PLANNER_2D_TEST
//    visualizer->publishBeliefMapFull(&beliefMap);
//    visualizer->publishLogOddsMapFull(&logOddsMap);
//    visualizer->publishTrueMap(&trueMap);
#endif

    delete stats;

#ifdef ENABLE_VISUALIZATION
    visualizer->render();
#endif

    return EXIT_SUCCESS;
}
