#include <octomap/octomap.h>

#include <ros/ros.h>

#include "../include/BeliefVoxel.h"
#include "../include/BeliefMap.h"
#include "../include/FakeRobot.hpp"
#include "../include/Visualizer.h"
#include "../include/Drone.h"
#include "../include/LogOddsMap.h"
#include "../include/Statistics.h"

TrueMap trueMap = TrueMap::generate(123); // TODO leave out seed value
BeliefMap beliefMap;
LogOddsMap logOddsMap;
FakeRobot<> robot(
        Parameters::Vec3Type(Parameters::voxelSize/2.f,
                             Parameters::voxelSize/2.f,
                             0),
#if defined(FAKE_2D)
        Parameters::Vec3Type(1, 0, 0),
#else
        Parameters::Vec3Type(0, 1, 0),
#endif
        trueMap);

std::vector<double> logOddsErrors, beliefErrors;
Statistics *stats;

void saveErrors(const char string[18], std::vector<double> vector);

void handleObservation(const Observation &observation)
{
    //trueMap.publish();
    beliefMap.update(observation, trueMap);
    logOddsMap.update(observation, trueMap);

    stats->update(logOddsMap, beliefMap, robot);

//    beliefErrors.push_back(beliefMap.error(trueMap));
//    logOddsErrors.push_back(logOddsMap.error(trueMap));

#if defined(FAKE_2D) || defined(FAKE_3D)
    if (!ros::ok())
        robot.stop();
#endif
}

void saveErrors(std::string filename, std::vector<double> errors)
{
    std::ofstream file(filename, std::ios_base::trunc);
    for (auto e : errors)
    {
        file << e << '\n';
    }
    file.close();
    ROS_INFO_STREAM("Saved errors to " << filename);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "SMAP");
    ros::Time::init();
    stats = new Statistics(trueMap);
    //trueMap.writeBinary("simple_tree.bt");

    Visualizer *visualizer = new Visualizer;
    //trueMap.subscribe(std::bind(&Visualizer::publishTrueMap, visualizer, std::placeholders::_1));
    //beliefMap.subscribe(std::bind(&Visualizer::publishBeliefMap, visualizer, std::placeholders::_1));
    logOddsMap.subscribe(std::bind(&Visualizer::publishLogOddsMap, visualizer, std::placeholders::_1));

#if defined(FAKE_2D)
    trueMap.subscribe(std::bind(&Visualizer::publishTrueMap2dSlice, visualizer, std::placeholders::_1, 0));
    robot.sensor().subscribe(std::bind(&Visualizer::publishStereoCameraSensor, visualizer, std::placeholders::_1));
    robot.registerObserver(&handleObservation);
    robot.run();
#elif defined(FAKE_3D)
    trueMap.subscribe(std::bind(&Visualizer::publishTrueMap, visualizer, std::placeholders::_1));
    robot.sensor().subscribe(std::bind(&Visualizer::publishStereoCameraSensor, visualizer, std::placeholders::_1));
    robot.registerObserver(&handleObservation);
    robot.run();
#else
    Drone drone;
    drone.registerObserver(&handleObservation);
    drone.run();
#endif

    //stats->update(logOddsMap, beliefMap, robot);
    stats->saveToFile("/home/eric/catkin_ws/src/smap/stats/stats.bag");

    saveErrors((std::string)"belief_errors.txt", beliefErrors);
    saveErrors((std::string)"logOdds_errors.txt", logOddsErrors);

    visualizer->render();

    return EXIT_SUCCESS;
}
