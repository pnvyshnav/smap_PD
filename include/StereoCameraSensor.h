#pragma once

#include <vector>

#include "Sensor.h"
#include "PixelSensor.h"
#include "Parameters.hpp"
#include "Observation.hpp"
#include "BeliefVoxel.h"
#include "TrueMap.h"
#include "BeliefMap.h"

class StereoCameraSensor : public Sensor
{
public:
    StereoCameraSensor(Parameters::Vec3Type &position,
                       Parameters::Vec3Type &orientation);

    virtual Observation observe(TrueMap &trueMap) const;

    std::vector<PixelSensor> pixels() const;

    struct InverseCauseModel
    {
        std::vector<Parameters::NumType> posteriorOnRay;
        std::vector<Parameters::NumType> posteriorInfinity;
        std::vector<octomap::OcTreeKey> voxelKeys;
    };

    InverseCauseModel computeInverseCauseModel(Measurement measurement, BeliefMap &beliefMap);

private:
	std::vector<PixelSensor> _pixelSensors;
};