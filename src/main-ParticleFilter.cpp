#include <octomap/octomap.h>

#include <ros/ros.h>

#include <ecl/time/stopwatch.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#undef MANY_STEPS

#include "../include/BeliefVoxel.h"
#include "../include/BeliefMap.h"
#include "../include/LogOddsMap.h"
#include "../include/GaussianProcessMap.h"
#include "../include/FakeRobot.hpp"
#include "../include/Visualizer.h"
#include "../include/Drone.h"
#include "../include/Statistics.hpp"

TrueMap trueMap = TrueMap::generateFromObstacles(std::vector<Box>{
        Box(7, 0, 0, 10, 1, 1)
});
BeliefMap beliefMap;
LogOddsMap logOddsMap;
GaussianProcessMap gaussianProcessMap;
auto *stats = new Statistics<>(trueMap);

FakeRobot<PixelSensor> robot(
        Parameters::Vec3Type(0.5, 0.5, 0.5),
        Parameters::Vec3Type(1, 0, 0),
        trueMap,
        beliefMap);

Observation allObservations;

int updated = 0;
void handleObservation(const Observation &observation)
{
    allObservations.append(observation);
    ROS_INFO("Handling observation %i...", ++updated);

    stats->registerMeasurements((int)observation.measurements().size());
    std::valarray<Parameters::NumType> rayLengths(observation.measurements().size());
    unsigned int i = 0;
    for (auto &measurement : observation.measurements())
    {
        rayLengths[i++] = measurement.value;
    }
    stats->registerRayStatistics(rayLengths.min(), rayLengths.max(), rayLengths.sum() / i);

    beliefMap.update(observation, trueMap);
    logOddsMap.update(observation, trueMap);

    stats->update(logOddsMap, beliefMap, gaussianProcessMap, robot);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "SMAP");
    ros::Time::init();

    // save initial particle state
    stats->update(logOddsMap, beliefMap, gaussianProcessMap, robot);

    auto *visualizer = new Visualizer;
//    robot.sensor().subscribe(std::bind(&Visualizer::publishStereoCameraSensor, visualizer, std::placeholders::_1));
    robot.registerObserver(&handleObservation);

    const int NUM_MEASUREMENTS = 20;
    for (int i = 0; i < NUM_MEASUREMENTS; ++i)
    {
        robot.runOnce();
    }

    for (int i = 0; i < 1; ++i)
    {
        visualizer->publishTrueMap(&trueMap);
        visualizer->publishObservation(&allObservations, false, true, 20);
        robot.publish();
        visualizer->sleep(200);
    }

    std::string homedir = getenv("HOME");
    stats->saveToFile(homedir + "/catkin_ws/src/smap/stats/particles/run02.bag");
    stats->reset();

    return EXIT_SUCCESS;
}
