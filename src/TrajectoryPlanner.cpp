#include <smap/smapStats.h>

#include "../include/TrajectoryPlanner.h"

//ts::BSpline spline1(1, 2, 3, TS_CLAMPED);
//std::vector<float> ctrlp1 = spline1.ctrlp();
//ctrlp1[0]  =  0.0f;
//ctrlp1[1]  = -0.9f;
//
//ctrlp1[2]  =  0.0f;
//ctrlp1[3]  =  0.0f;
//
//ctrlp1[4]  = -0.9f;
//ctrlp1[5]  =  0.0f;
//spline1.setCtrlp(ctrlp1);
//_splines.push_back(spline1);
//
//ts::BSpline spline2(2, 2, 4, TS_CLAMPED);
//std::vector<float> ctrlp2 = spline2.ctrlp();
//ctrlp2[0]  =  0.0f;
//ctrlp2[1]  = -0.9f;
//
//ctrlp2[2]  =  0.5f;
//ctrlp2[3]  = -0.1f;
//
//ctrlp2[4]  =  0.0f;
//ctrlp2[5]  =  0.0f;
//
//ctrlp2[6]  = -0.9f;
//ctrlp2[7]  =  0.0f;
//spline2.setCtrlp(ctrlp2);
//_splines.push_back(spline2);
//
//ts::BSpline spline3(2, 2, 4, TS_CLAMPED);
//std::vector<float> ctrlp3 = spline3.ctrlp();
//ctrlp3[0]  =  0.0f;
//ctrlp3[1]  = -0.9f;
//
//ctrlp3[2]  =  0.0f;
//ctrlp3[3]  =  0.4f;
//
//ctrlp3[4]  =  0.4f;
//ctrlp3[5]  =  0.0f;
//
//ctrlp3[6]  = -0.9f;
//ctrlp3[7]  =  0.0f;
//spline3.setCtrlp(ctrlp3);
//_splines.push_back(spline3);

TrajectoryPlanner::TrajectoryPlanner(TrueMap &trueMap, BeliefMap &beliefMap, LogOddsMap &logOddsMap)
    : _trueMap(trueMap), _beliefMap(beliefMap), _logOddsMap(logOddsMap), _stats(trueMap)
{
//    Trajectory trajectory({Point(0.0, -0.9), Point(0.5, -0.1), Point(0.0, 0.0), Point(-0.9, 0.0)});
//    trajectory.computeVelocities();
//    ROS_INFO("TOTAL ARC LENGTH: %f", trajectory.totalArcLength());
//    trajectory.computeTimeProfile();
//    ROS_INFO("TOTAL TIME:       %f", trajectory.totalTime());
//    trajectory.evaluate(0.5);

//    for (double x1 = 0.1; x1 < 0.7; x1 += 0.15)
//    {
//        for (double y1 = -0.2; y1 < 0.3; y1 += 0.15)
//        {
//            for (double x2 = -0.3; x2 < 0.2; x2 += 0.15)
//            {
//                for (double y2 = -0.2; y2 < 0.3; y2 += 0.15)
//                {
//                    Trajectory trajectory({Point(0.0, -0.9), Point(x1, y1), Point(x2, y2), Point(-0.9, 0.0)});
//                    trajectory.computeVelocities();
//                    ROS_INFO("TOTAL ARC LENGTH: %f", trajectory.totalArcLength());
//                    trajectory.computeTimeProfile();
//                    ROS_INFO("TOTAL TIME:       %f", trajectory.totalTime());
//                }
//            }
//        }
//
//    }

//    Trajectory trajectory2({Point(0.0, -0.9), Point(0.0, 0.4), Point(0.4, 0.0), Point(-0.9, 0.0)});
//    trajectory2.computeVelocities();
//    ROS_INFO("TOTAL ARC LENGTH: %f", trajectory2.totalArcLength());
//    trajectory2.computeTimeProfile();
//    ROS_INFO("TOTAL TIME:       %f", trajectory2.totalTime());
//    trajectory2.evaluate(0.5);
}

std::vector<Trajectory> TrajectoryPlanner::generateTrajectories() const
{
//    std::vector<Trajectory> trajectories;
//    double shift = Parameters::voxelSize/2.;
//    Trajectory t({Point(0.0 + shift, -0.9 + shift), Point(0.0 + shift, 0.9 + shift)}, 1);
//    t.computeVelocities();
//    t.computeTimeProfile();
//    trajectories.push_back(t);
//    return trajectories;

    Point lowerBound1( 0.1, -0.2), upperBound1(0.7, 0.35);
    Point lowerBound2(-0.3, -0.2), upperBound2(0.2, 0.35);

//    Point lowerBound1(-0.9, -0.9), upperBound1(0.7, 0.35);
//    Point lowerBound2(-0.9, -0.9), upperBound2(0.2, 0.35);

    const double stepSize = 0.25;

    unsigned int count = 0;
    double shift = Parameters::voxelSize / 2.; // make sure trajectory aligns at middle of voxel
    std::vector<Trajectory> trajectories;
    for (double x1 = lowerBound1.x; x1 <= upperBound1.x; x1 += stepSize)
    {
        for (double y1 = lowerBound1.y; y1 <= upperBound1.y; y1 += stepSize)
        {
            for (double x2 = lowerBound2.x; x2 <= upperBound2.x; x2 += stepSize)
            {
                for (double y2 = lowerBound2.y; y2 <= upperBound2.y; y2 += stepSize)
                {
                    Trajectory trajectory({Point(0.0 + shift, -0.9 + shift),
                                           Point(x1 + shift, y1 + shift),
                                           Point(x2 + shift, y2 + shift),
                                           Point(-0.9 + shift, 0.0 + shift)});
                    trajectory.computeVelocities();
                    ROS_INFO("TOTAL ARC LENGTH: %f", trajectory.totalArcLength());
                    trajectory.computeTimeProfile();
                    ROS_INFO("TOTAL TIME:       %f", trajectory.totalTime());
                    trajectory.saveProfile("trajectories/trajectory_" + std::to_string(count) + ".csv");
                    trajectories.push_back(trajectory);
                    ++count;
//                    if (count >= 10) // TODO remove?
//                    {
//                        ROS_INFO("%d trajectories were generated. Stopping.", (int)count);
//                        return trajectories;
//                    }
                }
            }
        }
    }
    ROS_INFO("%d trajectories were generated.", (int)count);
    return trajectories;
}


std::vector<Trajectory> TrajectoryPlanner::generateTrajectories(Point start, Point end,
                                                                VelocityPlanningParameters parameters) const
{
    const double stepSize = 0.25;

    unsigned int count = 0;
    std::vector<Trajectory> trajectories;
    // generate splines with 1 intermediary point
    for (double x = std::min(start.x, end.x); x <= std::max(start.x, end.x); x += stepSize)
    {
        for (double y = std::min(start.y, end.y); y <= std::max(start.y, end.y); y += stepSize)
        {
            Trajectory trajectory({start,
                                   Point(x, y),
                                   end});
            trajectory.computeVelocities();
            ROS_INFO("TOTAL ARC LENGTH: %f", trajectory.totalArcLength());
            trajectory.computeTimeProfile();
            ROS_INFO("TOTAL TIME:       %f", trajectory.totalTime());
            //trajectory.saveProfile("trajectories/trajectory_" + std::to_string(count) + ".csv");
            trajectories.push_back(trajectory);
//            ++count;
//                    if (count >= 10) // TODO remove?
//                    {
//                        ROS_INFO("%d trajectories were generated. Stopping.", (int)count);
//                        return trajectories;
//                    }
        }
    }
    ROS_INFO("%d trajectories were generated.", (int)count);
    return trajectories;
}

double TrajectoryPlanner::evaluate(Trajectory &trajectory, BeliefMap &map, const smap::smapStats &stats)
{
    double reachability = 1;
    auto startIndex = stats.trajectoryOccupanciesBelief.size() - stats.trajectoryVoxels;
    for (auto i = startIndex; i < stats.trajectoryOccupanciesBelief.size(); ++i)
    {
        reachability *= 1. - stats.trajectoryOccupanciesBelief[i];
    }
    reachability = 1. - reachability;
    // TODO compute reach - std(reach)
    return reachability;
}

Trajectory TrajectoryPlanner::replan(Point start, Point end, double startVelocity)
{
    VelocityPlanningParameters parameters;
    parameters.startVelocity = startVelocity;
    auto trajectories = generateTrajectories(start, end, parameters);
    double optimalCost = 0.;
    unsigned int i = 0, optimalIdx = 0;
    for (auto &candidate : trajectories)
    {
        _candidate = candidate;
        updateSubscribers();

        auto imaginaryMap = _beliefMap.copy();
        _simulationBot = new FakeRobot<PixelSensor>(Parameters::Vec3Type((float)start.x, (float)start.y, 0.05),
                                             Parameters::Vec3Type(0, 0, 0),
                                             _trueMap, imaginaryMap);
        _simulationBot->registerObserver(std::bind(&TrajectoryPlanner::_handleObservation,
                                                 this,
                                                 std::placeholders::_1));

        _simulationBot->setTrajectory(candidate);
        _simulationBot->run();
        auto cost = evaluate(candidate, imaginaryMap, _stats.stats());
        if (cost > optimalCost)
        {
            optimalCost = cost;
            optimalIdx = i;
        }
        _stats.reset();
        delete _simulationBot;
        ++i;
    }

    return trajectories[optimalIdx];
}

void TrajectoryPlanner::_handleObservation(const Observation &observation)
{
    _stats.update(_logOddsMap, _beliefMap, *_simulationBot);
}


