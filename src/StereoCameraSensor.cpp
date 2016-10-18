#include "../include/StereoCameraSensor.h"


StereoCameraSensor::StereoCameraSensor(Parameters::Vec3Type &position, Parameters::Vec3Type &orientation)
        : Sensor(position, orientation) {}

Observation StereoCameraSensor::observe(TrueMap &trueMap) const
{
    std::vector<Measurement> measurements;
    for (auto &sensor : _pixelSensors)
        measurements.push_back(sensor.observe(trueMap));
    return Observation(measurements);
}

std::vector<PixelSensor> StereoCameraSensor::pixels() const
{
    return _pixelSensors;
}