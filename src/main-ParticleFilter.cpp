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


int main(int argc, char **argv)
{
    ros::init(argc, argv, "SMAP");
    ros::Time::init();

    TrueMap trueMap =
    BeliefMap beliefMap;
    LogOddsMap logOddsMap;
    GaussianProcessMap gaussianProcessMap;
    FakeRobot<> robot(
            Parameters::Vec3Type(0.5, 1.5, 0.5),
            Parameters::Vec3Type(22.5, 1, 0),
            trueMap,
            beliefMap);
    Visualizer *visualizer = new Visualizer;

    Statistics<> *stats = new Statistics<>(trueMap);

    for (int i = 0; i < 1; ++i)
    {
        visualizer->publishTrueMap(&trueMap);
        visualizer->publishObservation(&allObservations, false, true, 20);
        visualizer->sleep(200);
    }

    ecl::StopWatch stopWatch;
    int cnt = 0;
    for (auto observation : loader.frames)
    {
        ROS_INFO("Updating frame %i out of %i (%i measurements)...", cnt,
                 (int)loader.frames.size(), (int)observation.measurements().size());
//            observation.translate(0.025, 0, 0); // traj1
//        observation.translate(0.015, 0, 0); // traj0
//        observation.scale(1.03, 1.03, 1.03); // traj0
        //TODO potentially dangerous if this measurement is not a voxel?
        robot.setPosition(observation.measurements().front().sensor.position);
//        robot.setOrientation(observation.measurements()[observation.measurements().size()/2].sensor.orientation);

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

//    stopWatch.restart();
//    gaussianProcessMap.update(observation);
//    stats->registerStepTimeGP(stopWatch.elapsed());

        stats->update(logOddsMap, beliefMap, gaussianProcessMap, robot);
        ++cnt;
    }

    std::string homedir = getenv("HOME");
    stats->saveToFile(homedir + "/catkin_ws/src/smap/stats/iclnuim/stats_traj" + std::to_string(trajectory) + ".bag");
    stats->reset();

    return EXIT_SUCCESS;
}
