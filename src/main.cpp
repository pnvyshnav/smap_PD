#include <octomap/octomap.h>

#include <ros/ros.h>

#include "../include/BeliefVoxel.h"
#include "../include/BeliefMap.h"
#include "../include/FakeRobot.hpp"
#include "../include/Visualizer.h"

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
    if (!ros::ok())
        robot.stop();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "SMAP");
    //trueMap.writeBinary("simple_tree.bt");

    Visualizer *visualizer = new Visualizer;
    //trueMap.subscribe(std::bind(&Visualizer::publishTrueMap, visualizer, std::placeholders::_1));
//    trueMap.subscribe(std::bind(&Visualizer::publishTrueMap2dSlice, visualizer, std::placeholders::_1, 0));
//    beliefMap.subscribe(std::bind(&Visualizer::publishBeliefMap, visualizer, std::placeholders::_1));
//    robot.sensor().subscribe(std::bind(&Visualizer::publishSensor, visualizer, std::placeholders::_1));

    robot.registerObserver(&handleObservation);
    robot.run();

    visualizer->publishTrueMap2dSlice(&trueMap);
    visualizer->publishBeliefMapFull(&beliefMap);
    visualizer->render();

    return EXIT_SUCCESS;
}