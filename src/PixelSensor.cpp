#include "../include/PixelSensor.h"
#include "../include/TruncatedGaussianDistribution.hpp"

#include <ros/console.h>
#include <cassert>

PixelSensor::PixelSensor(Parameters::Vec3Type position, Parameters::Vec3Type orientation)
	: FakeSensor(position, orientation)
{}

// TODO this is a hack
//Parameters::NumType scaledOccupancy(Parameters::NumType occupancy)
//{
//    // TrueMap occupancy values are either 0.4 (free) or 0.7 (occupied). // TODO this is incorrect, can be < 0.4
//    // The true mean is therefore 0.55.
//    auto dd = occupancy - 0.575; //TODO explain this value
//    return occupancy += dd * 1.7;
//}

Observation PixelSensor::observe(TrueMap &trueMap) const
{
	if (Parameters::deterministicSensorMeasurements)
	{
		octomap::point3d hitpoint;
		if (trueMap.castRay(_position, _orientation, hitpoint, false, Parameters::sensorRange))
		{
            return Measurement::voxel(std::make_shared<Sensor>(*this), (Parameters::NumType) _position.distance(hitpoint));
		}
	}
    else
    {
        std::vector<octomap::point3d> positions;
        octomap::point3d end_ray = _position + _orientation * Parameters::sensorRange;
        if (!trueMap.computeRay(_position, end_ray, positions) || positions.empty())
        {
            ROS_ERROR("Voxels on ray could not be computed.");
            return Measurement::hole(std::make_shared<Sensor>(*this));
        }
        else
        {
            unsigned int i = 0;
            for (auto &pos : positions)
            {
                ++i;
                QTrueVoxel voxel = trueMap.query(pos);
                if (voxel.type != GEOMETRY_VOXEL)
                    continue;
                auto sample = UniformDistribution::sample();
                // XXX this basically always yields the true cause voxel
                if (sample < std::round(voxel.node()->getOccupancy()))
                {
#ifdef LOG_DETAILS
                    ROS_INFO("Cause Voxel is %d/%d.  %f < %f", i, (int)positions.size(), sample,
                             scaledOccupancy(voxel.node()->getOccupancy()));
#endif
                    // voxel is the cause voxel
                    return _observationGivenCause(voxel);
                }
            }
        }
    }

#ifdef LOG_DETAILS
    ROS_WARN_STREAM("Sensor " << _position << " -> " << _orientation << " observed a hole.");
#endif
    return Measurement::hole(std::make_shared<Sensor>(*this));
}

Observation PixelSensor::observeImaginary(BeliefMap &beliefMap) const
{
    // TODO remove?
//    return Measurement::hole(std::make_shared<Sensor>(*this));

    std::vector<octomap::point3d> positions;
    octomap::point3d end_ray = _position + _orientation * Parameters::sensorRange;
    if (!beliefMap.computeRay(_position, end_ray, positions) || positions.empty())
    {
        ROS_ERROR("Voxels on ray could not be computed.");
        return Measurement::hole(std::make_shared<Sensor>(*this));
    }
    else
    {
        // pick the most likely cause voxel that maximizes eq. (27)
        std::valarray<double> scms(positions.size());
        unsigned int i = 0;
        double reachability = 1.;
        for (auto &pos : positions)
        {
            QBeliefVoxel voxel = beliefMap.query(pos);
            if (voxel.type != GEOMETRY_VOXEL)
            {
                scms[i] = 0;
                continue;
            }
            double scm = voxel.node()->getValue()->mean() * reachability;
            scms[i] = scm;
            reachability *= 1. - voxel.node()->getValue()->mean();
            ++i;
            if (i >= positions.size())
                break;
        }

        // probability that no voxel was occupied on ray
        double infinityCause = 1. - scms.sum();

        // find argmax(scms)
        unsigned int argmax = 0;
        for (unsigned int j = 1; j < scms.size(); ++j)
        {
            if (scms[j] > scms[argmax])
                argmax = j;
        }

        if (scms[argmax] < infinityCause)
        {
#ifdef LOG_DETAILS
            ROS_INFO("Sensor observed a hole.");
#endif
            return Measurement::hole(std::make_shared<Sensor>(*this));
        }

        return _observationGivenCause(beliefMap.query(positions[argmax]), true);
    }

#ifdef LOG_DETAILS
    ROS_WARN_STREAM("Sensor " << _position << " -> " << _orientation << " observed a hole.");
#endif
    return Measurement::hole(std::make_shared<Sensor>(*this));
}

Measurement PixelSensor::_observationGivenCause(QVoxel causeVoxel, bool deterministic) const
{
    if (causeVoxel.type != GEOMETRY_VOXEL)
    {
        return Measurement::hole(std::make_shared<Sensor>(*this));
    }
    assert(causeVoxel.type == GEOMETRY_VOXEL);
	Parameters::NumType deterministicRange = (Parameters::NumType) causeVoxel.position.distance(_position);
	if (deterministic)
		return Measurement::voxel(std::make_shared<Sensor>(*this), deterministicRange);
	auto tg = TruncatedGaussianDistribution(deterministicRange, Parameters::sensorNoiseStd,
                                            0, Parameters::sensorRange);
	return Measurement::voxel(std::make_shared<Sensor>(*this), tg.sample());
}

Parameters::NumType PixelSensor::likelihoodGivenCause(Measurement measurement, QVoxel causeVoxel) const
{
    if (causeVoxel.type == GEOMETRY_VOXEL)
    {
        if (measurement.geometry != GEOMETRY_VOXEL)
            return 0;

        auto z_mostLikely = _observationGivenCause(causeVoxel, true);
        // TODO try scenario where std dev is different from sensor model
        auto tg = TruncatedGaussianDistribution(z_mostLikely.value, Parameters::sensorNoiseStd, 0, Parameters::sensorRange);
        return tg.pdfValue(measurement.value);
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

FakeSensor::FakeSensor(Parameters::Vec3Type &position, Parameters::Vec3Type &orientation) : Sensor(position,
                                                                                                   orientation)
{}
