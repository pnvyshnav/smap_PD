#include "PixelSensor.h"

PixelSensor::PixelSensor(Parameters::Vec3Type direction, Parameters::Vec3Type position)
	: _position(position), _direction(direction) {}


Parameters::Vec3Type PixelSensor::direction() const
{
	return _direction;
}

void PixelSensor::setDirection(Parameters::Vec3Type direction)
{
	_direction = direction;
}


Parameters::Vec3Type PixelSensor::position() const
{
	return _position;
}

void PixelSensor::setPosition(Parameters::Vec3Type position)
{
	_position = position;
}


Parameters::NumType PixelSensor::observe(const TrueMap &trueMap) const
{
	if (Parameters::deterministicSensorMeasurements)
	{
		octomap::point3d hitpoint;
		if (!trueMap.castRay(_position, _direction, hitpoint, false, Parameters::sensorRange))
		{
			// no occupied cell was hit
			return std::numeric_limits<Parameters::NumType>::infinity();
		}
		return _position.distance(hitpoint);
	}

	//TODO implement non-deterministic case (add noise).
	return -1;
}
