#include "StereoCameraSensor.h"

Sensor::Sensor(Parameters::Vec3Type &position, Parameters::Vec3Type &orientation)
        : _position(position), _orientation(orientation) {}

Parameters::Vec3Type Sensor::position() const {
    return _position;
}

void Sensor::setPosition(const Parameters::Vec3Type &position) {
    _position = position;
}

Parameters::Vec3Type Sensor::orientation() const {
    return _orientation;
}

void Sensor::setOrientation(const Parameters::Vec3Type &orientation) {
    _orientation = orientation;
}


StereoCameraSensor::StereoCameraSensor(Parameters::Vec3Type &position, Parameters::Vec3Type &orientation)
        : Sensor(position, orientation) {}

Observation StereoCameraSensor::observe(const TrueMap &trueMap) const
{
    std::vector<Parameters::NumType> measurements;
    for (auto &sensor : _pixelSensors)
        measurements.push_back(sensor.observe(trueMap));
    return Observation(measurements);
}