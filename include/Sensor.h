#pragma once

#include "Parameters.hpp"
#include "Observation.hpp"
#include "TrueMap.h"

/**
 * Abstract class representing a sensor.
 */
class Sensor
{
public:
    Sensor(Parameters::Vec3Type &position,
           Parameters::Vec3Type &orientation);

    virtual Observation observe(TrueMap &trueMap) const = 0;

    Parameters::Vec3Type position() const;
    void setPosition(const Parameters::Vec3Type &position);

    Parameters::Vec3Type orientation() const;
    void setOrientation(const Parameters::Vec3Type &orientation);

protected:
    Parameters::Vec3Type _position;
    Parameters::Vec3Type _orientation;
};
