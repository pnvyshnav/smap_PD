#include <octomap/octomap.h>

#include <ros/ros.h>
#include <Eigen/Dense>

#include "../include/BeliefVoxel.h"
#include "../include/BeliefMap.h"
#include "../include/FakeRobot.hpp"
#include "../include/Visualizer.h"
#include "../include/Drone.h"
#include "../include/Statistics.hpp"
#include "../include/TrajectoryPlanner.h"
#include "../trajopt/MinSnapTrajectory.h"

TrueMap trueMap = TrueMap::generate(123); // use a fixed seed value
//    TrueMap trueMap = TrueMap::generateCorridor(); // use a fixed seed value

constexpr double shift = Parameters::voxelSize / 2.;

BeliefMap beliefMap;
FakeRobot<> robot(
        Parameters::Vec3Type(0.0 + shift, -0.9 + shift,
                             Parameters::zCenter),
        Parameters::Vec3Type(0, 1, 0),
        trueMap,
        beliefMap);

void handleObservation(const Observation &observation)
{
    beliefMap.update(observation, trueMap);

    trueMap.publish();
    beliefMap.publish();
    robot.publish();
    if (!ros::ok())
        robot.stop();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "SMAP");
    ros::Time::init();

    auto *visualizer = new Visualizer;

    trueMap.subscribe(std::bind(&Visualizer::publishTrueMap, visualizer, std::placeholders::_1));
    robot.subscribe(std::bind(&Visualizer::publishFakeRobot, visualizer, std::placeholders::_1, &trueMap));
    beliefMap.subscribe(std::bind(&Visualizer::publishBeliefMapFull, visualizer, std::placeholders::_1));
    robot.sensor().subscribe(std::bind(&Visualizer::publishStereoCameraSensor, visualizer, std::placeholders::_1));

    const MinSnapTrajectory trajectory(Point(0.0 + shift, -0.9 + shift), Point(-0.9 + shift, 0.0 + shift));
//    const MinSnapTrajectory trajectory(Point(0.35, -0.95), Point(0.35, 0.95), Point(), Point(), Point(), Point(), 4, 6);
    ROS_INFO("Min Snap DOF: %d", (int) trajectory.dof());
    TrajectoryPlanner *planner;

    robot.registerObserver(&handleObservation);

    // make initial observations
    for (int i = 0; i < 3; ++i)
        beliefMap.update(robot.observe(), trueMap);

    for (int i = 0; i < 20; ++i)
    {
        trueMap.publish();
        beliefMap.publish();
    }
    visualizer->sleep();

    planner = new TrajectoryPlanner(trajectory, trueMap, beliefMap);
    planner->costFunction().setKappa(1.);
    robot.setReplanningHandler(std::bind(&TrajectoryPlanner::replan, planner,
                                         std::placeholders::_1,
                                         std::placeholders::_2,
                                         std::placeholders::_3));
    planner->subscribe(std::bind(&Visualizer::publishTrajectoryPlanner, visualizer, std::placeholders::_1));
    beliefMap.update(robot.observe(), trueMap);
    auto optimized = planner->optimize();
    robot.setTrajectory(optimized);
    robot.run();

    delete planner;

    for (int i = 0; i < 20; ++i)
    {
        trueMap.publish();
        beliefMap.publish();
    }

    visualizer->render();

    return EXIT_SUCCESS;
}
