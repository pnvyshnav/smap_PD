#include "../include/TrueMap.h"
#include "../include/Statistics.h"

#include <smap/smapStats.h>



Statistics::Statistics(const TrueMap &trueMap) : _trueMap(trueMap)
{
    _nh = new ros::NodeHandle;
    _publisher = _nh->advertise<smap::smapStats>("stats", 1);
}

Statistics::~Statistics()
{
    delete _nh;
}

void Statistics::update(const LogOddsMap &logOddsMap, const BeliefMap &beliefMap)
{
    _msg.step = ++_step;
    _msg.errorLogOdds = logOddsMap.error(_trueMap);
    _msg.errorBelief = beliefMap.error(_trueMap);

    _msg.noiseStd = Parameters::sensorNoiseStd;
    _msg.ismIncrement = Parameters::invSensor_increment;
    _msg.ismRampSize = Parameters::invSensor_rampSize;
    _msg.ismTopSize = Parameters::invSensor_topSize;
    _msg.ismRampSlope = Parameters::invSensor_rampSlope;

    double evolutionLogOdds = 0;
    for (double error : _msg.errorLogOdds)
        evolutionLogOdds += std::abs(error);
    evolutionLogOdds /= _msg.errorLogOdds.size();
    _msg.errorEvolutionLogOdds.push_back(evolutionLogOdds);
    double evolutionBelief = 0;
    for (double error : _msg.errorBelief)
        evolutionBelief += std::abs(error);
    evolutionBelief /= _msg.errorBelief.size();
    _msg.errorEvolutionBelief.push_back(evolutionBelief);

    _msg.stdLogOdds.clear();
    for (auto &voxel : logOddsMap.voxels())
    {
        double pdf = voxel.node() == NULL ? Parameters::priorMean : voxel.node()->getOccupancy();
        // Bernoulli variance
        _msg.stdLogOdds.push_back(std::sqrt(pdf * (1. - pdf)));
    }

    _msg.stdBelief.clear();
    for (auto &voxel : beliefMap.voxels())
    {
        _msg.stdBelief.push_back(std::sqrt(voxel.node()->getValue()->variance()));
    }

    _publisher.publish(_msg);

    // TODO wait some time to ensure the data gets plotted
    ros::Rate publishing_rate(29);
    publishing_rate.sleep();
}
