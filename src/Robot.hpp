#pragma once

#include <vector>
#include <type_traits>

#include "Parameters.hpp"
#include "StereoCameraSensor.h"
#include "Observation.hpp"
#include "TrueMap.h"


class value;

template <class SENSOR>
class Robot
{
    static_assert(std::is_base_of<Sensor, SENSOR>::value, "SENSOR must inherit from Sensor");
public:
    Robot(Parameters::Vec3Type &position,
          Parameters::Vec3Type &orientation)
            : _position(position), _orientation(orientation),
              _sensor(position, orientation) {}

    Parameters::Vec3Type position() const
    {
        return _position;
    }

    void setPosition(const Parameters::Vec3Type &position)
    {
        _position = position;
        _sensor.setPosition(position);
    }

    Parameters::Vec3Type orientation() const
    {
        return _orientation;
    }

    void setOrientation(const Parameters::Vec3Type &orientation)
    {
        _orientation = orientation;
        _sensor.setOrientation(orientation);
    }

    Observation observe(const TrueMap &trueMap) const
    {
        return _sensor.observe(trueMap);
    }

private:
    Parameters::Vec3Type _position;
    Parameters::Vec3Type _orientation;
    SENSOR _sensor;
};