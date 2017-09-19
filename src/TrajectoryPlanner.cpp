//#include <smap/smapStats.h>
//
//#include "../include/TrajectoryPlanner.h"
//
//TrajectoryPlanner::TrajectoryPlanner(TrueMap &trueMap, BeliefMap &beliefMap, LogOddsMap &logOddsMap)
//    : _trueMap(trueMap), _beliefMap(beliefMap), _logOddsMap(logOddsMap), _stats(trueMap)
//{
////    BSplineTrajectory trajectory({Point(0.0, -0.9), Point(0.5, -0.1), Point(0.0, 0.0), Point(-0.9, 0.0)});
////    trajectory.computeVelocities();
////    ROS_INFO("TOTAL ARC LENGTH: %f", trajectory.totalArcLength());
////    trajectory.computeTimeProfile();
////    ROS_INFO("TOTAL TIME:       %f", trajectory.totalTime());
////    trajectory.evaluate(0.5);
//
////    for (double x1 = 0.1; x1 < 0.7; x1 += 0.15)
////    {
////        for (double y1 = -0.2; y1 < 0.3; y1 += 0.15)
////        {
////            for (double x2 = -0.3; x2 < 0.2; x2 += 0.15)
////            {
////                for (double y2 = -0.2; y2 < 0.3; y2 += 0.15)
////                {
////                    BSplineTrajectory trajectory({Point(0.0, -0.9), Point(x1, y1), Point(x2, y2), Point(-0.9, 0.0)});
////                    trajectory.computeVelocities();
////                    ROS_INFO("TOTAL ARC LENGTH: %f", trajectory.totalArcLength());
////                    trajectory.computeTimeProfile();
////                    ROS_INFO("TOTAL TIME:       %f", trajectory.totalTime());
////                }
////            }
////        }
////
////    }
//
////    BSplineTrajectory trajectory2({Point(0.0, -0.9), Point(0.0, 0.4), Point(0.4, 0.0), Point(-0.9, 0.0)});
////    trajectory2.computeVelocities();
////    ROS_INFO("TOTAL ARC LENGTH: %f", trajectory2.totalArcLength());
////    trajectory2.computeTimeProfile();
////    ROS_INFO("TOTAL TIME:       %f", trajectory2.totalTime());
////    trajectory2.evaluate(0.5);
//}
//
//std::vector<BSplineTrajectory> TrajectoryPlanner::generateTrajectories()
//{
//    std::vector<BSplineTrajectory> trajectories;
//    double shift = Parameters::voxelSize / 2.; // make sure trajectory aligns at middle of voxel
//
//#ifdef ONLY_HANDCRAFTED_TRAJECTORIES
//    Point start(0.0 + shift, -0.9 + shift);
//    Point end(-0.9 + shift, 0.0 + shift);
//    BSplineTrajectory trajectory0({start, Point(0 + shift, 0 + shift), end}, 1);
//    trajectory0.computeVelocities();
//    trajectory0.computeTimeProfile();
//    trajectory0.saveProfile("handcrafted_trajectories/trajectory_0.csv");
//    trajectories.push_back(trajectory0);
//
//    BSplineTrajectory trajectory1({start, Point(0.5 + shift, -0.1 + shift), Point(0 + shift, 0 + shift), end}, 2);
//    trajectory1.computeVelocities();
//    trajectory1.computeTimeProfile();
//    trajectory1.saveProfile("handcrafted_trajectories/trajectory_1.csv");
//    trajectories.push_back(trajectory1);
//
//    BSplineTrajectory trajectory2({start, Point(0 + shift, 0.4 + shift), Point(0.4 + shift, 0 + shift), end}, 2);
//    trajectory2.computeVelocities();
//    trajectory2.computeTimeProfile();
//    trajectory2.saveProfile("handcrafted_trajectories/trajectory_2.csv");
//    trajectories.push_back(trajectory2);
//    return trajectories;
//#endif
//
////    Point lowerBound1( 0.1, -0.2), upperBound1(0.7, 0.35);
////    Point lowerBound2(-0.3, -0.2), upperBound2(0.2, 0.35);
//
//    Point lowerBound1(-0.9, -0.9), upperBound1(0.7, 0.35);
////    Point lowerBound2(-0.9, -0.9), upperBound2(0.2, 0.35);
//
//    const double stepSize = 0.15;
//
//    unsigned int count = 0;
//    for (double x1 = lowerBound1.x; x1 <= upperBound1.x; x1 += stepSize)
//    {
//        for (double y1 = lowerBound1.y; y1 <= upperBound1.y; y1 += stepSize)
//        {
////            for (double x2 = lowerBound2.x; x2 <= upperBound2.x; x2 += stepSize)
////            {
////                for (double y2 = lowerBound2.y; y2 <= upperBound2.y; y2 += stepSize)
////                {
//                    BSplineTrajectory trajectory({Point(0.0 + shift, -0.9 + shift),
//                                           Point(x1 + shift, y1 + shift),
////                                           Point(x2 + shift, y2 + shift),
//                                           Point(-0.9 + shift, 0.0 + shift)});
//                    trajectory.computeVelocities();
//                    ROS_INFO("TOTAL ARC LENGTH: %f", trajectory.totalArcLength());
//                    trajectory.computeTimeProfile();
//                    ROS_INFO("TOTAL TIME:       %f", trajectory.totalTime());
//                    if (trajectory.totalTime() > Parameters::SimulationFinalTime)
//                    {
//                        ROS_WARN("BSplineTrajectory's total time is too long (%f > %f).",
//                                 trajectory.totalTime(),
//                                 Parameters::SimulationFinalTime);
//                        continue;
//                    }
//                    trajectory.saveProfile("handcrafted_trajectories/trajectory_" + std::to_string(count) + ".csv");
//                    trajectories.push_back(trajectory);
//                    ++count;
////                    if (count >= 10) // TODO remove?
////                    {
////                        ROS_INFO("%d trajectories were generated. Stopping.", (int)count);
////                        return trajectories;
////                    }
////                }
////            }
//        }
//    }
//    ROS_INFO("%d trajectories were generated.", (int)count);
//    return trajectories;
//}
//
//
//std::vector<BSplineTrajectory> TrajectoryPlanner::generateTrajectories(Point start, Point end,
//                                                                VelocityPlanningParameters parameters)
//{
//    std::vector<BSplineTrajectory> trajectories;
//
//    const double stepSize = 0.25;
//    const double scaling = 2.0;
//
//    unsigned int count = 0;
//
//    // scale control point sampling rectangle beyond start / end
//    double scaledWidth = std::abs(end.x - start.x) / 2.0 * scaling;
//    double scaledHeight = std::abs(end.y - start.y) / 2.0 * scaling;
//    double midX = (start.x + end.x) / 2.0;
//    double midY = (start.y + end.y) / 2.0;
//
//    // generate splines with 1 intermediary point
//    for (double x = midX - scaledWidth; x <= midX + scaledWidth; x += stepSize)
//    {
//        for (double y = midY - scaledHeight; y <= midY + scaledHeight; y += stepSize)
//        {
//            BSplineTrajectory trajectory({start, Point(x, y), end});
//            trajectory._end = end;
//            trajectory.computeVelocities();
//            if (!trajectory.isValid())
//            {
//                ROS_WARN("Skipped invalid trajectory");
//                continue;
//            }
//            trajectory.computeTimeProfile();
//            ROS_INFO("Generated trajectory %i \t(arc length: %f, \ttime: %f)",
//                     (int)count, trajectory.totalArcLength(), trajectory.totalTime());
//            //trajectory.saveProfile("trajectories/trajectory_" + std::to_string(count) + ".csv");
//            trajectories.push_back(trajectory);
//            ++count;
////                    if (count >= 10) // TODO remove?
////                    {
////                        ROS_INFO("%d trajectories were generated. Stopping.", (int)count);
////                        return trajectories;
////                    }
//        }
//    }
//    ROS_INFO("%d trajectories were generated.", (int)count);
//    return trajectories;
//}
//
//double TrajectoryPlanner::evaluate(BSplineTrajectory &trajectory, BeliefMap &map, const smap::smapStats &stats)
//{
//    double reachability = 1.;
//    auto startIndex = stats.trajectoryOccupanciesBelief.size() - stats.trajectoryVoxels;
//    double reachLeft = 1., reachRight = 1.;
//    for (auto i = startIndex; i < stats.trajectoryOccupanciesBelief.size(); ++i)
//    {
//        auto beliefVoxel = _beliefMap.query(Parameters::Vec3Type(stats.trajectoryVoxelX[i%stats.trajectoryVoxels], stats.trajectoryVoxelY[i%stats.trajectoryVoxels], 0.05));
//        if (!beliefVoxel.node())
//            continue;
//
//        double var = std::pow(stats.trajectoryStdDevsBelief[i], 2.);
//        double imOcc = stats.trajectoryOccupanciesBelief[i];
////        if (std::abs(imOcc - 0.5) < 0.1)
////            imOcc = 0.2;
//        double reOcc = _beliefMap.getVoxelMean(beliefVoxel);
//        if (std::abs(reOcc - 0.5) < 0.1)
//            reOcc = 0.1;
//
//        double mean = 1. - 0.5 * (imOcc + reOcc); // do we take the belief map or imaginary reachability?
//        //ROS_INFO("BSplineTrajectory 1-mean: %f", mean);
//        reachability *= 1. - reOcc;
//        double meanSq = std::pow(mean, 2.);
//        reachLeft *= var + meanSq;
//        reachRight *= meanSq;
//    }
//    double stdReachability = std::sqrt(reachLeft - reachRight);
//    //reachability = 1. - reachability;
//    // TODO compute reach - std(reach)
//    ROS_INFO("Reachability: %f \tStd(Reachability): %f", reachability, stdReachability);
//    double kappa = 3.5; // stdReachability currently doesn't have a big influence
//    return reachability - kappa * stdReachability;
//}
//
//BSplineTrajectory TrajectoryPlanner::replan(Point start, Point end, double startVelocity)
//{
//    VelocityPlanningParameters parameters;
//    parameters.startVelocity = startVelocity;
//    auto trajectories = generateTrajectories(start, end, parameters);
//    double optimalUtility = 0.;
//    unsigned int i = 0, optimalIdx = 0;
//    for (auto &candidate : trajectories)
//    {
//        _candidate = candidate;
//        updateSubscribers();
//
//        _imaginaryMap = BeliefMap(_beliefMap);
////        auto further = candidate.evaluate(0.4);
////        _simulationBot = new FakeRobot<PixelSensor>(Parameters::Vec3Type((float)further.point.x, (float)further.point.y, 0.05),
////                                             Parameters::Vec3Type(1, 0, 0),
////                                             _trueMap, _imaginaryMap);
//        _simulationBot = new FakeRobot<PixelSensor>(Parameters::Vec3Type((float)start.x, (float)start.y, 0.05),
//                                                    Parameters::Vec3Type(1, 0, 0),
//                                                    _trueMap, _imaginaryMap);
//        _simulationBot->setObservationMode(OBSERVE_ONLY_IMAGINARY);
//        _simulationBot->registerObserver(std::bind(&TrajectoryPlanner::_handleObservation,
//                                                 this,
//                                                 std::placeholders::_1));
//
//        _simulationBot->setTrajectory(candidate);
//        _simulationBot->_step = 20;
//        _simulationBot->run();
//        auto utility = evaluate(candidate, _imaginaryMap, _stats.stats());
//        ROS_INFO("BSplineTrajectory %d has utility: %.10f", (int)i, utility);
//        if (utility > optimalUtility)
//        {
//            optimalUtility = utility;
//            optimalIdx = i;
//        }
//        _stats.reset();
//        delete _simulationBot;
//        ++i;
//    }
//
//    ROS_INFO("BSplineTrajectory %i has the best utility.", (int)optimalIdx);
//    return trajectories[optimalIdx];
//}
//
//void TrajectoryPlanner::_handleObservation(const Observation &observation)
//{
//    _imaginaryMap.update(observation, _trueMap);
//    _stats.update(_logOddsMap, _imaginaryMap, *_simulationBot);
//}
//
//TrajectoryData TrajOpt::_evaluateCandidate() const {
//    return TrajectoryData();
//}
//
//BSplineTrajectory TrajectoryPlanner::generateInitialDirectTrajectory(Point start, Point end)
//{
//    BSplineTrajectory trajectory({start, Point((start.x + end.x) / 2., (start.y + end.y) / 2.), end});
//    trajectory.computeVelocities();
//    trajectory.computeTimeProfile();
//    return trajectory;
//}
//
//