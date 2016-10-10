//
// Created by eric on 10/9/16.
//

#include "PixelSensor.h"

PixelSensor::PixelSensor(Parameters::NumType angle) : _angle(angle) {}

Parameters::NumType PixelSensor::angle() const {
    return _angle;
}

void PixelSensor::setAngle(Parameters::NumType _angle) {
    PixelSensor::_angle = _angle;
}
