#pragma once

#include <vector>
#include <octomap/OcTree.h>

#include "parameters.hpp"

class Observation {
public:
    Observation(const std::vector<Parameters::NumType> &measurements);

    const std::vector<Parameters::NumType> &measurements() const;

private:
    std::vector<Parameters::NumType> _measurements;
};

class RangeSensor {
public:
    virtual Observation observe(const octomap::OcTree &trueMap) const = 0;

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
    RobotSensor(const Parameters::Vec3Type &_position, const Parameters::Vec3Type &_orientation, RangeSensor *_sensor);

    const Parameters::Vec3Type &position() const;
    void setPosition(const Parameters::Vec3Type &_position);

    const Parameters::Vec3Type &orientation() const;
    void setOrientation(const Parameters::Vec3Type &_orientation);

    Observation observe(const octomap::OcTree &trueMap) const;

private:
    Parameters::Vec3Type _position;
    Parameters::Vec3Type _orientation;
    RangeSensor *_sensor;
};
