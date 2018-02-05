#pragma once

#include <vector>

#include "Sensor.h"
#include "PixelSensor.h"
#include "Parameters.h"
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
    virtual Observation observeImaginary(BeliefMap &beliefMap) const;

    std::vector<PixelSensor> pixels() const;

    void setPosition(const Parameters::Vec3Type &position);
    void setOrientation(const Parameters::Vec3Type &orientation);
    void resetOrientation(const Parameters::Vec3Type &orientation);

private:
	std::vector<PixelSensor> _pixelSensors;
};