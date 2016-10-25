#include "Sensor.h"

Sensor::Sensor(Parameters::Vec3Type &position, Parameters::Vec3Type &orientation) : _position(position),
                                                                                    _orientation(orientation)
{}

Parameters::Vec3Type Sensor::position() const
{
    return _position;
}

void Sensor::setPosition(const Parameters::Vec3Type &position)
{
    _position = position;
    updateVisualization();
}

Parameters::Vec3Type Sensor::orientation() const
{
    return _orientation;
}

void Sensor::setOrientation(const Parameters::Vec3Type &orientation)
{
    _orientation = orientation;
    updateVisualization();
}

InverseCauseModel
Sensor::computeInverseCauseModel(Measurement measurement, TrueMap &trueMap, BeliefMap &beliefMap) const
{
    InverseCauseModel icm;
    octomap::KeyRay ray;
    beliefMap.computeRayKeys(position(), orientation() * Parameters::sensorRange + position(), ray);
    octomap::KeyRay rayTrueMap;
    trueMap.computeRayKeys(position(), orientation() * Parameters::sensorRange + position(), rayTrueMap);
    unsigned int j = 0;
    icm.ray.reserve(ray.size());
    std::vector<QTrueVoxel> causeVoxels;
    for (auto &key : ray)
    {
        icm.ray.push_back(key);
        causeVoxels.push_back(trueMap.query(key));
        ++j;
        if (j >= ray.size())
            break;
    }
    icm.rayLength = ray.size();
    auto bouncingProbabilities = beliefMap.bouncingProbabilitiesOnRay(ray);
    auto reachingProbabilities = beliefMap.reachingProbabilitiesOnRay(ray, bouncingProbabilities);
    std::valarray<Parameters::NumType> prior = bouncingProbabilities * reachingProbabilities;

    ROS_INFO("Found %i cause voxels.", (int)causeVoxels.size());

    std::valarray<Parameters::NumType> likelihood(ray.size());
    for (unsigned int i = 0; i < ray.size(); ++i)
        likelihood[i] = likelihoodGivenCause(measurement, causeVoxels[i]);
    icm.posteriorOnRay = likelihood * prior;

    const unsigned int end = ray.size() - 1;

    Parameters::NumType bouncingProbabilityFromInfinity = 1;
    Parameters::NumType reachingProbabilityFromInfinity = (Parameters::NumType) ((1. - bouncingProbabilities[end]) *
                                                                                 reachingProbabilities[end]);

    auto causeProbabilityFromInfinityPrior = bouncingProbabilityFromInfinity * reachingProbabilityFromInfinity;

    Parameters::NumType likelihoodGivenInfinity = likelihoodGivenCause(measurement, QTrueVoxel::hole());

    icm.posteriorInfinity = likelihoodGivenInfinity * causeProbabilityFromInfinityPrior;

   //TODO  assert(std::abs(prior.sum() + causeProbabilityFromInfinityPrior - (Parameters::NumType)1.) < (Parameters::NumType)1e-5);

    auto eta = icm.posteriorOnRay.sum() + icm.posteriorInfinity;
    if (eta < 1e-10)
    {
        ROS_WARN("Inverse Cause Model: eta = %g", eta);
    }
    else
    {
        icm.posteriorOnRay /= eta;
        icm.posteriorInfinity /= eta;
    }

    assert(std::abs(icm.posteriorOnRay.sum() + icm.posteriorInfinity - 1.) < 1e-10);

    return icm;
}