#pragma once

#include <valarray>

#include "Parameters.hpp"
#include "TrueMap.h"
#include "Observation.hpp"
#include "BeliefMap.h"
#include "Sensor.h"


/**
 * @brief Represents a range sensor that only computes a single ray.
 */
class PixelSensor : public Sensor
{
    friend class StereoCameraSensor;
public:
    PixelSensor(Parameters::Vec3Type position, Parameters::Vec3Type orientation);

    /**
     * @brief Simulates a range sensor measurement given the true map. 
     * 
     * @param trueMap The true occupancy map.
     * @return @see Observation.
     */
    Observation observe(TrueMap &trueMap) const;

    /**
     * @brief Computes the likelihood of the measurement given the true voxel that caused the measurement.
     * @param measurement The measurement.
     * @param causeVoxel The true cause voxel.
     * @return Likelihood between 0 and 1.
     */
    Parameters::NumType likelihoodGivenCause(Measurement measurement, QTrueVoxel causeVoxel) const;

private:
    /**
     * @brief Simulate observation of a voxel.
     * @param causeVoxel The voxel to be observed.
     * @param deterministic If false, the measurement becomes noisy.
     * @return Simulated observation measurement.
     */
    Measurement _observationGivenCause(QTrueVoxel causeVoxel,
                                       bool deterministic = Parameters::deterministicSensorMeasurements) const;
};
