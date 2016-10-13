#pragma once

#include <vector>
#include <octomap/OcTree.h>

#include "Parameters.hpp"
#include "Observation.hpp"
#include "PixelSensor.h"
#include "TrueMap.h"

class Sensor
{
public:
    Sensor(Parameters::Vec3Type &position,
                   Parameters::Vec3Type &orientation);

    virtual Observation observe(const TrueMap &trueMap) const = 0;

    Parameters::Vec3Type position() const;
    void setPosition(const Parameters::Vec3Type &position);

    Parameters::Vec3Type orientation() const;
    void setOrientation(const Parameters::Vec3Type &orientation);

protected:
    Parameters::Vec3Type _position;
    Parameters::Vec3Type _orientation;
};

class StereoCameraSensor : public Sensor
{
public:
    StereoCameraSensor(Parameters::Vec3Type &position,
                       Parameters::Vec3Type &orientation);

    virtual Observation observe(const TrueMap &trueMap) const;

private:
	std::vector<PixelSensor> _pixelSensors;
};