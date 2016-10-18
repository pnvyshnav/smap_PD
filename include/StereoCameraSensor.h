#pragma once

#include <vector>

#include "Sensor.h"
#include "PixelSensor.h"
#include "Parameters.hpp"
#include "Observation.hpp"
#include "BeliefVoxel.h"
#include "TrueMap.h"

class StereoCameraSensor : public Sensor
{
public:
    StereoCameraSensor(Parameters::Vec3Type &position,
                       Parameters::Vec3Type &orientation);

    virtual Observation observe(TrueMap &trueMap) const;

    std::vector<PixelSensor> pixels() const;

    // TODO update signature and implement!
    void InverseCauseModel(Measurement measurement, BeliefVoxel beliefVoxel);

private:
	std::vector<PixelSensor> _pixelSensors;
};