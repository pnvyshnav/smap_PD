#pragma once

#include <vector>

#include "Sensor.h"
#include "PixelSensor.h"
#include "Parameters.hpp"
#include "Observation.hpp"
#include "BeliefVoxel.h"
#include "TrueMap.h"
#include "BeliefMap.h"

class StereoCameraSensor : public FakeSensor
{
public:
    StereoCameraSensor(Parameters::Vec3Type position,
                       Parameters::Vec3Type orientation);

    virtual Observation observe(TrueMap &trueMap) const;

    std::vector<PixelSensor> pixels() const;

    /**
     * @brief Computes the likelihood of the measurement given the true voxel that caused the measurement.
     * @param measurement The measurement.
     * @param causeVoxel The true cause voxel.
     * @return Likelihood between 0 and 1.
     */
    Parameters::NumType likelihoodGivenCause(Measurement measurement, QVoxel causeVoxel) const;

    void setPosition(const Parameters::Vec3Type &position);
    void setOrientation(const Parameters::Vec3Type &orientation);

private:
	std::vector<PixelSensor> _pixelSensors;
};