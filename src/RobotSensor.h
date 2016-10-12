#pragma once

#include <vector>
#include <octomap/OcTree.h>

#include "Parameters.hpp"
#include "Sensor.h"
#include "Observation.hpp"


class Robot {
public:
    Robot(const Parameters::Vec3Type &_position, const Parameters::Vec3Type &_orientation, RangeSensor *_sensor);

    const Parameters::Vec3Type &position() const;
    void setPosition(const Parameters::Vec3Type &_position);

    const Parameters::Vec3Type &orientation() const;
    void setOrientation(const Parameters::Vec3Type &_orientation);

private:
    Parameters::Vec3Type _position;
    Parameters::Vec3Type _orientation;
};

class RobotSensor {
public:
    RobotSensor(const Parameters::Vec3Type &_position, const Parameters::Vec3Type &_orientation);

    const Robot &robot() const;
    const RangeSensor &sensor() const;

    Observation observe(const octomap::OcTree &trueMap) const;

private:
    Robot _robot;
    RangeSensor _sensor;
};
