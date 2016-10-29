#pragma once

#include <vector>
#include <type_traits>

#include "Parameters.hpp"
#include "StereoCameraSensor.h"
#include "Observation.hpp"
#include "TrueMap.h"
#include "Robot.hpp"


/**
 * Representation of a robot in a given pose and having a sensor.
 * Provided SENSOR template has to behave like {@see Sensor}.
 */
template <class SENSOR = StereoCameraSensor>
class FakeRobot : public Robot
{
public:
    FakeRobot(Parameters::Vec3Type position,
          Parameters::Vec3Type orientation,
          TrueMap &trueMap)
            : _position(position), _orientation(orientation),
              _sensor(position, orientation),
              _trueMap(trueMap)
    {}

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

    Observation observe()
    {
        return _sensor.observe(_trueMap);
    }

    SENSOR &sensor()
    {
        return _sensor;
    }

    void run()
    {
        _stopRequested = false;
        for (auto rad = 0.; !_stopRequested; rad += 8. * M_PI / 180.)
        {
            setOrientation(Parameters::Vec3Type(std::cos(rad), std::sin(rad), 0));
            Robot::publishObservation(observe());
        }
    }

    void stop()
    {
        _stopRequested = true;
    }

private:
    Parameters::Vec3Type _position;
    Parameters::Vec3Type _orientation;
    TrueMap _trueMap;
    SENSOR _sensor;
    bool _stopRequested;
};