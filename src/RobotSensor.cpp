//
// Created by eric on 10/9/16.
//

#include "RobotSensor.h"


const Parameters::Vec3Type &RobotSensor::position() const {
return _position;
}

void RobotSensor::setPosition(const Parameters::Vec3Type &_position) {
    RobotSensor::_position = _position;
}

const Parameters::Vec3Type &RobotSensor::orientation() const {
    return _orientation;
}

void RobotSensor::setOrientation(const Parameters::Vec3Type &_orientation) {
    RobotSensor::_orientation = _orientation;
}

RobotSensor::RobotSensor(const Parameters::Vec3Type &_position, const Parameters::Vec3Type &_orientation,
                         RangeSensor *_sensor) : _position(_position), _orientation(_orientation), _sensor(_sensor) {}

Observation RobotSensor::observe(const octomap::OccupancyOcTreeBase &trueMap) const {
    return Observation(std::vector<Parameters::NumType>());
}

const std::vector<Parameters::NumType> &Observation::measurements() const {
    return _measurements;
}

Observation::Observation(const std::vector<Parameters::NumType> &measurements) : _measurements(measurements) {}

const Parameters::Vec3Type &RangeSensor::position() const {
    return _position;
}

void RangeSensor::setPosition(const Parameters::Vec3Type &_position) {
    RangeSensor::_position = _position;
}

const Parameters::Vec3Type &RangeSensor::orientation() const {
    return _orientation;
}

void RangeSensor::setOrientation(const Parameters::Vec3Type &_orientation) {
    RangeSensor::_orientation = _orientation;
}
