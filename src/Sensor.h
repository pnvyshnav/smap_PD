#pragma once

#include <vector>
#include <octomap/OcTree.h>

#include "Parameters.hpp"
#include "Observation.hpp"

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

class StereoSensor : public RangeSensor {
public:
	virtual Observation observe(const octomap::OcTree &trueMap) const;
}