#pragma once

#include <vector>

#include "Sensor.h"
#include "PixelSensor.h"
#include "Parameters.hpp"
#include "Observation.hpp"
#include "TrueMap.h"

class StereoCameraSensor : public Sensor
{
public:
    StereoCameraSensor(Parameters::Vec3Type &position,
                       Parameters::Vec3Type &orientation);

    virtual Observation observe(TrueMap &trueMap) const;

private:
	std::vector<PixelSensor> _pixelSensors;
};