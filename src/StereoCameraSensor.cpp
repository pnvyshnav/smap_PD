#include "../include/StereoCameraSensor.h"


StereoCameraSensor::StereoCameraSensor(Parameters::Vec3Type &position, Parameters::Vec3Type &orientation)
        : Sensor(position, orientation) {}

Observation StereoCameraSensor::observe(const TrueMap &trueMap) const
{
    std::vector<Parameters::NumType> measurements;
    for (auto &sensor : _pixelSensors)
        measurements.push_back(sensor.observe(trueMap));
    return Observation(measurements);
}