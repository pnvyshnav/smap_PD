#include "../include/PixelSensor.h"
#include "../include/TruncatedGaussianDistribution.hpp"

#include <ros/console.h>
#include <cassert>

PixelSensor::PixelSensor(Parameters::Vec3Type direction, Parameters::Vec3Type position)
	: _position(position), _direction(direction)
{}


Parameters::Vec3Type PixelSensor::direction() const
{
	return _direction;
}

void PixelSensor::setDirection(Parameters::Vec3Type direction)
{
	_direction = direction.normalized();
}


Parameters::Vec3Type PixelSensor::position() const
{
	return _position;
}

void PixelSensor::setPosition(Parameters::Vec3Type position)
{
	_position = position;
}


Measurement PixelSensor::observe(TrueMap &trueMap) const
{
	if (Parameters::deterministicSensorMeasurements)
	{
		octomap::point3d hitpoint;
		if (trueMap.castRay(_position, _direction, hitpoint, false, Parameters::sensorRange))
		{
            return Measurement::voxel(_position.distance(hitpoint));
		}
	}
    else
    {
        std::vector<octomap::point3d> positions;
        octomap::point3d end_ray = _position + _direction * Parameters::sensorRange;
        if (!trueMap.computeRay(_position, end_ray, positions) || positions.empty())
        {
            ROS_ERROR("Voxels on ray could not be computed.");
            return Measurement::hole();
        }
        else
        {
            for (auto &pos : positions)
            {
                auto voxel = trueMap.query(pos);
                if (UniformDistribution::sample() < voxel.occupancy)
                {
                    // voxel is the cause voxel
                    return _observationGivenCause(voxel);
                }
            }
        }
    }

    return Measurement::hole();
}

Measurement PixelSensor::_observationGivenCause(QVoxel &causeVoxel, bool deterministic) const
{
    assert(causeVoxel.type == GEOMETRY_VOXEL);
	Parameters::NumType deterministicRange = (Parameters::NumType) causeVoxel.position.distance(_position);
	if (deterministic)
		return Measurement::voxel(deterministicRange);
	auto tg = TruncatedGaussianDistribution(deterministicRange, Parameters::sensorNoiseStd,
                                            (Parameters::NumType) 0., Parameters::sensorRange);
	return Measurement::voxel(tg.sample());
}

Parameters::NumType PixelSensor::likelihoodGivenCause(Measurement measurement, QVoxel &causeVoxel) const
{
    if (causeVoxel.type == GEOMETRY_VOXEL)
    {
        if (measurement.geometry != GEOMETRY_VOXEL)
            return 0;

        auto z_mostLikely = _observationGivenCause(causeVoxel, true);
        auto tg = TruncatedGaussianDistribution(z_mostLikely.value, Parameters::sensorNoiseStd, 0, Parameters::sensorRange);
        return tg.pdfValue(measurement.value);
    }
    else if (causeVoxel.type == GEOMETRY_HOLE)
    {
        return measurement.geometry == GEOMETRY_HOLE;
    }
    else
    {
        if (measurement.geometry == GEOMETRY_HOLE)
            return 0;

        return (Parameters::NumType) (1. / Parameters::sensorRange);
    }
}
