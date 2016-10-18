#include "../include/PixelSensor.h"
#include "../include/TruncatedGaussianDistribution.hpp"

#include <ros/console.h>

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


Parameters::NumType PixelSensor::observe(TrueMap &trueMap) const
{
	if (Parameters::deterministicSensorMeasurements)
	{
		octomap::point3d hitpoint;
		if (!trueMap.castRay(_position, _direction, hitpoint, false, Parameters::sensorRange))
		{
			// no occupied cell was hit
			return std::numeric_limits<Parameters::NumType>::infinity();
		}
		return (Parameters::NumType) _position.distance(hitpoint);
	}
    else
    {
        std::vector<octomap::point3d> positions;
        octomap::point3d end_ray = _position + _direction * Parameters::sensorRange;
        if (!trueMap.computeRay(_position, end_ray, positions) || positions.empty())
        {
            ROS_ERROR("Voxels on ray could not be computed.");
            return std::numeric_limits<Parameters::NumType>::infinity();
        }
        else
        {
            for (auto &pos : positions)
            {
                auto voxel = trueMap.query(pos);
                if (UniformDistribution::sample() < voxel.occupancy)
                {
                    // voxel is the cause voxel
                    return _observeGivenCause(voxel);
                }
            }

            // no voxel has been sampled, so pick first reached voxel
            octomap::point3d causePos = positions.front();
            TrueVoxel cause = trueMap.query(causePos);
            return _observeGivenCause(cause);
        }
    }
}

Parameters::NumType PixelSensor::_observeGivenCause(TrueVoxel &causeVoxel) const
{
	Parameters::NumType deterministicRange = (Parameters::NumType) causeVoxel.position.distance(_position);
	if (Parameters::deterministicSensorMeasurements)
		return deterministicRange;
	auto tg = TruncatedGaussianDistribution(deterministicRange, Parameters::sensorNoiseStd,
                                            (Parameters::NumType) 0., Parameters::sensorRange);
	return tg.sample();
}
