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

TrajectoryPlanner::TrajectoryPlanner(TrueMap &trueMap, BeliefMap &beliefMap)
    : _trueMap(trueMap), _beliefMap(beliefMap)
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
    unsigned int count = 0;
    std::vector<Trajectory> trajectories;
    for (double x1 = 0.1; x1 < 0.7; x1 += 0.25)
    {
        for (double y1 = -0.2; y1 <= 0.35; y1 += 0.25)
        {
            for (double x2 = -0.3; x2 < 0.2; x2 += 0.25)
            {
                for (double y2 = -0.2; y2 <= 0.35; y2 += 0.25)
                {
                    Trajectory trajectory({Point(0.0, -0.9), Point(x1, y1), Point(x2, y2), Point(-0.9, 0.0)});
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

void TrajectoryPlanner::evaluate(Trajectory &trajectory, BeliefMap &map, const smap::smapStats &stats)
{
    // TODO implement
}


