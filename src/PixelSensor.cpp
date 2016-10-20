#include "../include/PixelSensor.h"
#include "../include/TruncatedGaussianDistribution.hpp"

#include <ros/console.h>
#include <cassert>

PixelSensor::PixelSensor(Parameters::Vec3Type position, Parameters::Vec3Type orientation)
	: Sensor(position, orientation)
{}

Observation PixelSensor::observe(TrueMap &trueMap) const
{
	if (Parameters::deterministicSensorMeasurements)
	{
		octomap::point3d hitpoint;
		if (trueMap.castRay(_position, _orientation, hitpoint, false, Parameters::sensorRange))
		{
            return Measurement::voxel(this, (Parameters::NumType) _position.distance(hitpoint));
		}
	}
    else
    {
        std::vector<octomap::point3d> positions;
        octomap::point3d end_ray = _position + _orientation * Parameters::sensorRange;
        if (!trueMap.computeRay(_position, end_ray, positions) || positions.empty())
        {
            ROS_ERROR("Voxels on ray could not be computed.");
            return Measurement::hole(this);
        }
        else
        {
            for (auto &pos : positions)
            {
                QTrueVoxel voxel = trueMap.query(pos);
                if (UniformDistribution::sample() < voxel.node->getOccupancy())
                {
                    // voxel is the cause voxel
                    return _observationGivenCause(voxel);
                }
            }
        }
    }

    return Measurement::hole(this);
}

Measurement PixelSensor::_observationGivenCause(QTrueVoxel causeVoxel, bool deterministic) const
{
    assert(causeVoxel.type == GEOMETRY_VOXEL);
	Parameters::NumType deterministicRange = (Parameters::NumType) causeVoxel.position.distance(_position);
	if (deterministic)
		return Measurement::voxel(this, deterministicRange);
	auto tg = TruncatedGaussianDistribution(deterministicRange, Parameters::sensorNoiseStd,
                                            (Parameters::NumType) 0., Parameters::sensorRange);
	return Measurement::voxel(this, tg.sample());
}

Parameters::NumType PixelSensor::likelihoodGivenCause(Measurement measurement, QTrueVoxel causeVoxel) const
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