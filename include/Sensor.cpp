#include "Sensor.h"

Sensor::Sensor(Parameters::Vec3Type &position, Parameters::Vec3Type &orientation)
        : _position(position), _orientation(orientation)
{}

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