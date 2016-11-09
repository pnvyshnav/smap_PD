#include <octomap/octomap.h>

#include <ros/ros.h>

#include "../include/BeliefVoxel.h"
#include "../include/BeliefMap.h"
#include "../include/FakeRobot.hpp"
#include "../include/Visualizer.h"
#include "../include/Drone.h"

TrueMap trueMap = TrueMap::generate();
BeliefMap beliefMap;
FakeRobot<> robot(
        Parameters::Vec3Type(Parameters::voxelSize/2.f,
                             Parameters::voxelSize/2.f,
                             0),
        Parameters::Vec3Type(1, 0, 0),
        trueMap);

void handleObservation(const Observation &observation)
{
    trueMap.publish();
    beliefMap.update(observation);
#ifdef FAKE_2D
    if (!ros::ok())
        robot.stop();
#endif
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "SMAP");
    //trueMap.writeBinary("simple_tree.bt");

    Visualizer *visualizer = new Visualizer;
    //trueMap.subscribe(std::bind(&Visualizer::publishTrueMap, visualizer, std::placeholders::_1));
    beliefMap.subscribe(std::bind(&Visualizer::publishBeliefMapFull, visualizer, std::placeholders::_1));

#ifdef FAKE_2D
    trueMap.subscribe(std::bind(&Visualizer::publishTrueMap2dSlice, visualizer, std::placeholders::_1, 0));
    robot.sensor().subscribe(std::bind(&Visualizer::publishSensor, visualizer, std::placeholders::_1));
    robot.registerObserver(&handleObservation);
    robot.run();
#else
    Drone drone;
    drone.registerObserver(&handleObservation);
    drone.run();
#endif

    visualizer->publishTrueMap2dSlice(&trueMap);
    visualizer->publishBeliefMapFull(&beliefMap);
    visualizer->render();

    return EXIT_SUCCESS;
}