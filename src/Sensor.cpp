#include "../include/Sensor.h"
#include "../include/TruncatedGaussianDistribution.hpp"

octomap::KeyRay Sensor::_keyRay;

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
    updateSubscribers();
}

Parameters::Vec3Type Sensor::orientation() const
{
    return _orientation;
}

void Sensor::setOrientation(const Parameters::Vec3Type &orientation)
{
    _orientation = orientation;
    updateSubscribers();
}

Parameters::NumType Sensor::range() const
{
    return _range;
}

Parameters::NumType
Sensor::likelihoodGivenCause(const Measurement &measurement, const QVoxel &causeVoxel)
{
    if (causeVoxel.type == GEOMETRY_VOXEL)
    {
        if (measurement.geometry != GEOMETRY_VOXEL)
            return 0;

#ifdef FAKE_2D
        auto tg = TruncatedGaussianDistribution(causeVoxel.position.distance(measurement.sensor.position), Parameters::sensorNoiseStd,//TODO sensorNoiseStd range-dependent
                                                0, measurement.sensor.range); // TODO truncated?
        return tg.pdfValue(measurement.value);
#else
        //  return std::abs(measurement.value - causeVoxel.position.distance(measurement.sensor.position)) < Parameters::voxelSize;
        auto tg = TruncatedGaussianDistribution(causeVoxel.position.distance(measurement.sensor.position), Parameters::sensorNoiseStd,//TODO sensorNoiseStd range-dependent
                                                0, measurement.sensor.range * 2.); // TODO truncated too early?
        return tg.pdfValue(measurement.value);
#endif
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

InverseCauseModel
Sensor::inverseCauseModel(const Measurement &measurement, const BeliefMap &beliefMap)
{

    InverseCauseModel icm;
//    ROS_INFO("computeInverseCauseModel: position %f %f %f", _position.x(), _position.y(), _position.z());
    if (!beliefMap.computeRayKeys(measurement.sensor.position,
                                  measurement.sensor.orientation * Parameters::sensorRange + measurement.sensor.position,
                                  _keyRay))
    {
        ROS_WARN("Sensor::inverseCauseModel: Computing ray keys failed.");
        return InverseCauseModel::EMPTY;
    }

    if (_keyRay.size() == 0)
    {
        return InverseCauseModel::EMPTY;
    }

    unsigned int j = 0;
    icm.ray.reserve(_keyRay.size());
    std::vector<QVoxel> causeVoxels;
    for (auto &key : _keyRay)
    {
        icm.ray.push_back(key);
        causeVoxels.push_back(beliefMap.query(key));
        ++j;
        if (j >= _keyRay.size())
            break;
    }
    icm.rayLength = (unsigned int) _keyRay.size();
    auto bouncingProbabilities = beliefMap.bouncingProbabilitiesOnRay(_keyRay);
    auto reachingProbabilities = beliefMap.reachingProbabilitiesOnRay(_keyRay, bouncingProbabilities);
    std::valarray<Parameters::NumType> prior = bouncingProbabilities * reachingProbabilities;

#ifdef LOG_DETAILS
    ROS_INFO("Found %i cause voxels.", (int)causeVoxels.size());
#endif

    std::valarray<Parameters::NumType> likelihood(_keyRay.size());
    for (unsigned int i = 0; i < _keyRay.size(); ++i)
        likelihood[i] = likelihoodGivenCause(measurement, causeVoxels[i]);
    icm.posteriorOnRay = likelihood * prior;


    const unsigned int end = (const unsigned int) (_keyRay.size() - 1);

    Parameters::NumType bouncingProbabilityFromInfinity = 1;
    Parameters::NumType reachingProbabilityFromInfinity = (Parameters::NumType) ((1. - bouncingProbabilities[end]) *
                                                                                 reachingProbabilities[end]);

    auto causeProbabilityFromInfinityPrior = bouncingProbabilityFromInfinity * reachingProbabilityFromInfinity;
    assert(!std::isnan(causeProbabilityFromInfinityPrior));

    Parameters::NumType likelihoodGivenInfinity = likelihoodGivenCause(measurement, QBeliefVoxel::hole());

    icm.posteriorInfinity = likelihoodGivenInfinity * causeProbabilityFromInfinityPrior;

    if (std::abs(prior.sum() + causeProbabilityFromInfinityPrior - 1) >= 1e-10)
    {
        ROS_WARN("Sensor::inverseCauseModel: New assertion fired.");
        return InverseCauseModel::EMPTY;
    }

    auto eta = icm.posteriorOnRay.sum() + icm.posteriorInfinity;
    if (eta == 0.)
    {
//        ROS_WARN("Inverse Cause Model: eta = %g", eta);
        // TODO correct behavior?
        return InverseCauseModel::EMPTY;
    }
    else
    {
        icm.posteriorOnRay /= eta;
        icm.posteriorInfinity /= eta;
    }

    if (std::abs(icm.posteriorOnRay.sum() + icm.posteriorInfinity - 1.) >= 1e-10)
    {
//        ROS_WARN("ICM assertion failed. Test: %g < 1e-20", std::abs(icm.posteriorOnRay.sum() + icm.posteriorInfinity - 1.));
        return InverseCauseModel::EMPTY;
    }

    icm.empty = false; // otherwise empty by default
    return icm;
}
