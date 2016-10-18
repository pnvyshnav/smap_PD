#pragma once

#include "Parameters.hpp"
#include "TrueMap.h"
#include "Observation.hpp"


/**
 * @brief Represents a range sensor that only computes a single ray.
 */
class PixelSensor
{
public:
    PixelSensor(Parameters::Vec3Type direction, Parameters::Vec3Type position);

    /**
     * @brief Normalized vector describing the orientation of the pixel sensor.
     */
    Parameters::Vec3Type direction() const;
    void setDirection(Parameters::Vec3Type direction);


    /**
     * @brief Global position of the pixel sensor.
     */
    Parameters::Vec3Type position() const;
    void setPosition(Parameters::Vec3Type position);


    /**
     * @brief Simulates a range sensor measurement given the true map. 
     * 
     * @param trueMap The true occupancy map.
     * @return @see Measurement.
     */
    Measurement observe(TrueMap &trueMap) const;

    /**
     * @brief Computes the likelihood of the measurement given the true voxel that caused the measurement.
     * @param measurement The measurement.
     * @param causeVoxel The true cause voxel.
     * @return Likelihood between 0 and 1.
     */
    Parameters::NumType likelihoodGivenCause(Measurement measurement, QVoxel &causeVoxel) const;

private:
    Parameters::Vec3Type _direction;
    Parameters::Vec3Type _position;

    /**
     * @brief Simulate observation of a voxel.
     * @param causeVoxel The voxel to be observed.
     * @param deterministic If false, the measurement becomes noisy.
     * @return Simulated observation measurement.
     */
    Measurement _observationGivenCause(QVoxel &causeVoxel,
                                       bool deterministic = Parameters::deterministicSensorMeasurements) const;
};
