#pragma once

#include <valarray>

#include "Parameters.h"
#include "TrueMap.h"
#include "Observation.hpp"
#include "BeliefMap.h"
#include "Sensor.h"

class FakeSensor : public Sensor
{
public:
    FakeSensor(Parameters::Vec3Type &position,
               Parameters::Vec3Type &orientation);

    virtual Observation observe(TrueMap &trueMap) const = 0;
};


/**
 * @brief Represents a range sensor that only computes a single ray.
 */
class PixelSensor : public FakeSensor
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
     * @brief Simulates an imaginary range sensor measurement given the belief map.
     *
     * @param beliefMap The belief map.
     * @return @see Observation.
     */
    Observation observeImaginary(BeliefMap &beliefMap) const;

    /**
     * @brief Computes the likelihood of the measurement given the true voxel that caused the measurement.
     * @param measurement The measurement.
     * @param causeVoxel The true cause voxel.
     * @return Likelihood between 0 and 1.
     */
    Parameters::NumType likelihoodGivenCause(Measurement measurement, QVoxel causeVoxel) const;

private:
    /**
     * @brief Simulate observation of a voxel.
     * @param causeVoxel The voxel to be observed.
     * @param deterministic If false, the measurement becomes noisy.
     * @return Simulated observation measurement.
     */
    Measurement _observationGivenCause(QVoxel causeVoxel,
                                       bool deterministic = Parameters::deterministicSensorMeasurements) const;
};
