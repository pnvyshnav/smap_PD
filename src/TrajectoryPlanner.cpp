#include "../include/TrajectoryPlanner.h"

TrajectoryPlanner::TrajectoryPlanner(TrueMap &trueMap, BeliefMap &beliefMap)
    : _trueMap(trueMap), _beliefMap(beliefMap)
{
//    Trajectory trajectory({Point(0.0, -0.9), Point(0.5, -0.1), Point(0.0, 0.0), Point(-0.9, 0.0)});
//    trajectory.computeVelocities();
//    ROS_INFO("TOTAL ARC LENGTH: %f", trajectory.totalArcLength());
//    trajectory.computeTimeProfile();
//    ROS_INFO("TOTAL TIME:       %f", trajectory.totalTime());
//    trajectory.evaluate(0.5);
    for (double x1 = 0.1; x1 < 0.7; x1 += 0.15)
    {
        for (double y1 = -0.2; y1 < 0.3; y1 += 0.15)
        {
            for (double x2 = -0.3; x2 < 0.2; x2 += 0.15)
            {
                for (double y2 = -0.2; y2 < 0.3; y2 += 0.15)
                {
                    Trajectory trajectory({Point(0.0, -0.9), Point(x1, y1), Point(x2, y2), Point(-0.9, 0.0)});
                    trajectory.computeVelocities();
                    ROS_INFO("TOTAL ARC LENGTH: %f", trajectory.totalArcLength());
                    trajectory.computeTimeProfile();
                    ROS_INFO("TOTAL TIME:       %f", trajectory.totalTime());
                }
            }
        }

    }

//    Trajectory trajectory2({Point(0.0, -0.9), Point(0.0, 0.4), Point(0.4, 0.0), Point(-0.9, 0.0)});
//    trajectory2.computeVelocities();
//    ROS_INFO("TOTAL ARC LENGTH: %f", trajectory2.totalArcLength());
//    trajectory2.computeTimeProfile();
//    ROS_INFO("TOTAL TIME:       %f", trajectory2.totalTime());
//    trajectory2.evaluate(0.5);

    std::exit(0);
}
