#include <octomap/octomap.h>

#include <ros/ros.h>
#include <Eigen/Dense>

#include <ecl/time/stopwatch.hpp>

#include "../include/BeliefVoxel.h"
#include "../include/BeliefMap.h"
#include "../include/FakeRobot.hpp"
#include "../include/Visualizer.h"
#include "../include/Drone.h"
#include "../include/LogOddsMap.h"
#include "../include/Statistics.hpp"
#include "../include/TrajectoryPlanner.h"
#include "../include/PointCloud.h"
#include "../trajopt/MinSnapTrajectory.h"

#ifdef REAL_3D
TrueMap trueMap = TrueMap::generateFromPointCloud("/home/eric/catkin_ws/src/smap/dataset/V1_01_easy/data.ply");
#else
TrueMap trueMap = TrueMap::generate(123); // use a fixed seed value
//    TrueMap trueMap = TrueMap::generateCorridor(); // use a fixed seed value
#endif

constexpr double shift = Parameters::voxelSize / 2.;

BeliefMap beliefMap;
LogOddsMap logOddsMap;
FakeRobot<> robot(
        Parameters::Vec3Type(0.0 + shift, -0.9 + shift,
                             Parameters::zCenter),
        Parameters::Vec3Type(0, 1, 0),
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
        stats->saveToFile("/home/eric/catkin_ws/src/smap/stats/stats_real3d_"
                          + std::to_string(updated) + ".bag");
#ifdef SLIM_STATS
        stats->reset();
#endif
    }
#endif
    ++updated;

#if defined(FAKE_2D) || defined(FAKE_3D)
    trueMap.publish();
    beliefMap.publish();
    robot.publish();
    if (!ros::ok())
        robot.stop();
#endif
}

int main(int argc, char **argv)
{
//    Eigen::VectorXd cs(6);
//    cs  << 0, 1, 2, 3, 4, 5;
//    MinSnapTrajectory::Polynomial p(5, cs);
//    std::cout << p.str() << "  p(2) = " << p.evaluate(2) << std::endl;
//    std::cout << p.derivative().str() << "  p'(2) = " << p.derivative().evaluate(2) << std::endl;
//    std::cout << p.derivative().derivative().str() << "  p''(2) = " << p.derivative().derivative().evaluate(2) << std::endl;
//
//    return 0;
//    PointCloud cloud;
//    cloud.loadPly("/home/eric/catkin_ws/src/smap/dataset/V1_01_easy/groundtruth_pcl.ply");
//    cloud.visualize();

    ros::init(argc, argv, "SMAP");
    ros::Time::init();

    stats = new Statistics<>(trueMap);

    Visualizer *visualizer = new Visualizer;


//#ifdef ENABLE_VISUALIZATION
    trueMap.subscribe(std::bind(&Visualizer::publishTrueMap, visualizer, std::placeholders::_1));
//    for (int i = 0; i < 40; ++i)
//    {
//        trueMap.publish();
//        beliefMap.publish();
//    }
//#ifndef REAL_3D
    robot.subscribe(std::bind(&Visualizer::publishFakeRobot, visualizer, std::placeholders::_1, &trueMap));
//#endif
//    beliefMap.subscribe(std::bind(&Visualizer::publishBeliefMapFull, visualizer, std::placeholders::_1));
//    beliefMap.subscribe(std::bind(&Visualizer::publishBeliefMap, visualizer, std::placeholders::_1));

//    trueMap.subscribe(std::bind(&Visualizer::publishTrueMap2dSlice, visualizer, std::placeholders::_1, 0));
//    trueMap.subscribe(std::bind(&Visualizer::publishTrueMap, visualizer, std::placeholders::_1));


//
//    visualizer->render();
//    return 0;

//#ifdef PLANNER_2D_TEST
//    planner->subscribe(std::bind(&Visualizer::publishTrajectoryPlanner, visualizer, std::placeholders::_1));
//#endif
//#ifdef FAKE_2D
//    beliefMap.subscribe(std::bind(&Visualizer::publishBeliefMapFull, visualizer, std::placeholders::_1));
//#else
//    // todo reactivate
    beliefMap.subscribe(std::bind(&Visualizer::publishBeliefMapFull, visualizer, std::placeholders::_1));
    logOddsMap.subscribe(std::bind(&Visualizer::publishLogOddsMapFull, visualizer, std::placeholders::_1));
//#endif
    robot.sensor().subscribe(std::bind(&Visualizer::publishStereoCameraSensor, visualizer, std::placeholders::_1));
//#endif

    const MinSnapTrajectory trajectory(Point(0.0 + shift, -0.9 + shift), Point(-0.9 + shift, 0.0 + shift));
    ROS_INFO("Min Snap DOF: %d", (int)trajectory.dof());
    TrajectoryPlanner *planner;

    robot.registerObserver(&handleObservation);

    // make initial observations
    for (int i = 0; i < 3; ++i)
        beliefMap.update(robot.observe(), trueMap);

    for (int i = 0; i < 20; ++i)
    {
        trueMap.publish();
        beliefMap.publish();
        robot.publish();
    }
    visualizer->sleep();

            planner = new TrajectoryPlanner(trajectory, trueMap, beliefMap);
            planner->costFunction().setKappa(2.);
            robot.setReplanningHandler(std::bind(&TrajectoryPlanner::replan, planner,
                                                 std::placeholders::_1,
                                                 std::placeholders::_2,
                                                 std::placeholders::_3));
    planner->subscribe(std::bind(&Visualizer::publishTrajectoryPlanner, visualizer, std::placeholders::_1));
            beliefMap.update(robot.observe(), trueMap);
//        #if defined(REPLANNING)
//            robot.setReplanningHandler(std::bind(&TrajectoryPlanner::replan, planner,
//                                         std::placeholders::_1,
//                                         std::placeholders::_2,
//                                         std::placeholders::_3));
//            //robot.setTrajectory(planner.replan(Point(0.05, -0.95), Point(-0.95, 0.05), 0.0));
//            robot.setTrajectory(TrajectoryPlanner::generateInitialDirectTrajectory(Point(0.05, -0.95), Point(-0.95, 0.05)));
//            robot.run();
//            stats->saveToFile("replanning/replanning.bag");
//        #else
            auto optimized = planner->optimize();
//            planner->replan(Point(0.0 + shift, -0.9 + shift), Point(-0.9 + shift, 0.0 + shift), Point(1,4));
            robot.setTrajectory(optimized);
            robot.run();

            stats->registerReplanningIterations(planner->replanningIterations());
//            stats->saveToFile(
//                    "/home/wal/catkin_ws/src/smap/stats/replanning_tunnel/lcb_"
//                    + std::to_string(kappa) + "_" + std::to_string(round) + ".bag");
//            stats->reset();
//            beliefMap.reset();
            delete planner;
//        }
//    }

    for (int i = 0; i < 20; ++i)
    {
        trueMap.publish();
        beliefMap.publish();
    }

//            unsigned int splineId = 0;
//            for (auto &trajectory : planner.generateTrajectories())
//            {
//                ROS_INFO("Evaluating spline %d...", (int)splineId);
//                beliefMap.reset();
//                logOddsMap.reset();
//                robot.setTrajectory(trajectory);
//                robot.run();
//                planner.evaluate(trajectory, beliefMap, stats->stats());
//        #ifdef ONLY_HANDCRAFTED_TRAJECTORIES
//                stats->saveToFile("handcrafted_trajeval/trajectory_" + std::to_string(splineId) + ".bag");
//        #else
//                stats->saveToFile("time trajeval mlCause one pixel singlePoint/trajectory_" + std::to_string(splineId) + ".bag");
//        #endif
//                stats->reset();
//                ++splineId;
//            }
//        #endif
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
    drone.runOffline("/home/eric/catkin_ws/src/smap/dataset/V1_01_easy/droneflight.bag");
#endif

#ifndef PLANNER_2D_TEST
    //    visualizer->publishBeliefMapFull(&beliefMap);
//    visualizer->publishLogOddsMapFull(&logOddsMap);
//    visualizer->publishTrueMap(&trueMap);
    stats->saveToFile("/home/eric/catkin_ws/src/smap/stats/stats.bag");
#endif

    delete stats;

//    delete planner;

#ifdef ENABLE_VISUALIZATION
    visualizer->render();
#endif

    return EXIT_SUCCESS;
}
