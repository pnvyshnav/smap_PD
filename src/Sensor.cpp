#include "../include/Sensor.h"
#include "../include/TruncatedGaussianDistribution.hpp"

Sensor::Sensor(Parameters::Vec3Type position, Parameters::Vec3Type orientation, Parameters::NumType range)
        : _range(range), _position(position), _orientation(orientation)
{

}

Sensor::Sensor(const Sensor &sensor) : _position(sensor._position),
                   _orientation(sensor._orientation),
                   _range(sensor._range)
{

}

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

Parameters::NumType Sensor::likelihoodGivenCause(Measurement measurement, QVoxel causeVoxel) const
{
    if (causeVoxel.type == GEOMETRY_VOXEL)
    {
        if (measurement.geometry != GEOMETRY_VOXEL)
            return 0;

        auto tg = TruncatedGaussianDistribution(measurement.value, Parameters::sensorNoiseStd * measurement.value * 0.1 ,//TODO sensorNoiseStd range-dependent
                                                0, _range * 2.0); // TODO truncated?
        return tg.pdfValue(causeVoxel.position.distance(_position));
    }
    else if (causeVoxel.type == GEOMETRY_HOLE)
    {
        return (int)(measurement.geometry == GEOMETRY_HOLE);
    }
    else
    {
        if (measurement.geometry == GEOMETRY_HOLE)
            return 0;

        return (Parameters::NumType) (1. / Parameters::sensorRange);
    }
}

InverseCauseModel *Sensor::computeInverseCauseModel(Measurement measurement, BeliefMap &beliefMap) const
{
    auto *icm = new InverseCauseModel;

    octomap::KeyRay ray;
    if (!beliefMap.computeRayKeys(_position, _orientation * _range * 2.0 + _position, ray)) //TODO remove *2.0
    {
        ROS_WARN("Compute ray keys failed.");
        delete icm;
        return NULL;
    }

    if (ray.size() == 0)
    {
        delete icm;
        return NULL;
    }

    unsigned int j = 0;
    icm->ray.reserve(ray.size());
    std::vector<QVoxel> causeVoxels;
    for (auto &key : ray)
    {
        icm->ray.push_back(key);
        causeVoxels.push_back(beliefMap.query(key));
        ++j;
        if (j >= ray.size())
            break;
    }
    icm->rayLength = ray.size();
    auto bouncingProbabilities = beliefMap.bouncingProbabilitiesOnRay(ray);
    auto reachingProbabilities = beliefMap.reachingProbabilitiesOnRay(ray, bouncingProbabilities);
    std::valarray<Parameters::NumType> prior = bouncingProbabilities * reachingProbabilities;

#ifdef LOG_DETAILS
    ROS_INFO("Found %i cause voxels.", (int)causeVoxels.size());
#endif

    std::valarray<Parameters::NumType> likelihood(ray.size());
    for (unsigned int i = 0; i < ray.size(); ++i)
        likelihood[i] = likelihoodGivenCause(measurement, causeVoxels[i]);
    icm->posteriorOnRay = likelihood * prior;


    const unsigned int end = ray.size() - 1;

    Parameters::NumType bouncingProbabilityFromInfinity = 1;
    Parameters::NumType reachingProbabilityFromInfinity = (Parameters::NumType) ((1. - bouncingProbabilities[end]) *
                                                                                 reachingProbabilities[end]);

    auto causeProbabilityFromInfinityPrior = bouncingProbabilityFromInfinity * reachingProbabilityFromInfinity;
    assert(!std::isnan(causeProbabilityFromInfinityPrior));

    Parameters::NumType likelihoodGivenInfinity = likelihoodGivenCause(measurement, QBeliefVoxel::hole());

    icm->posteriorInfinity = likelihoodGivenInfinity * causeProbabilityFromInfinityPrior;

    auto eta = icm->posteriorOnRay.sum() + icm->posteriorInfinity;
    if (eta < 1e-10)
    {
        ROS_WARN("Inverse Cause Model: eta = %g", eta);
    }
    else
    {
        icm->posteriorOnRay /= eta;
        icm->posteriorInfinity /= eta;
    }

    if (!(std::abs(icm->posteriorOnRay.sum() + icm->posteriorInfinity - 1.) < 1e-10))
    {
        ROS_WARN("ICM assertion failed. Test: %g < 1e-10", std::abs(icm->posteriorOnRay.sum() + icm->posteriorInfinity - 1.));
        delete icm;
        return NULL;
    }

    return icm;
}

Parameters::NumType Sensor::range() const
{
    return _range;
}
