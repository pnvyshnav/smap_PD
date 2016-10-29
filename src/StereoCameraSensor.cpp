#include "../include/StereoCameraSensor.h"


StereoCameraSensor::StereoCameraSensor(Parameters::Vec3Type &position, Parameters::Vec3Type &orientation)
        : Sensor(position, orientation)
{
    // TODO initialize with more than 1 pixel
    _pixelSensors.push_back(PixelSensor(position, orientation));
}

Observation StereoCameraSensor::observe(TrueMap &trueMap) const
{
    std::vector<Measurement> measurements;
    for (auto &sensor : _pixelSensors)
        measurements.push_back(sensor.observe(trueMap).measurements().front());
    return Observation(measurements);
}

std::vector<PixelSensor> StereoCameraSensor::pixels() const
{
    return _pixelSensors;
}

Parameters::NumType StereoCameraSensor::likelihoodGivenCause(Measurement measurement, QVoxel causeVoxel) const
{
    return measurement.sensor->likelihoodGivenCause(measurement, causeVoxel);
}

void StereoCameraSensor::setPosition(const Parameters::Vec3Type &position)
{
    auto diff = this->position() - position;
    Sensor::setPosition(position);
    for (auto &pixel : _pixelSensors)
    {
        pixel._position += diff;
    }
}

void StereoCameraSensor::setOrientation(const Parameters::Vec3Type &orientation)
{
    // TODO handle real pixel transformations
    Sensor::setOrientation(orientation);
    for (auto &pixel : _pixelSensors)
    {
        pixel._orientation = orientation;
    }
}

