#pragma once

#include <vector>
#include <type_traits>
#include <unordered_set>

#include <ecl/time/stopwatch.hpp>

#include "Parameters.hpp"
#include "StereoCameraSensor.h"
#include "Observation.hpp"
#include "TrueMap.h"
#include "Robot.hpp"
#include "LogOddsMap.h"

enum ObservationMode
{
    OBSERVE_ONLY_REAL,
    OBSERVE_ONLY_IMAGINARY,
    OBSERVE_BOTH
};

/**
 * Representation of a robot in a given pose and having a sensor.
 * Provided SENSOR template has to behave like {@see Sensor}.
 */
template <class SENSOR = StereoCameraSensor>
class FakeRobot : public Robot, public Observable
{
public:
    FakeRobot(Parameters::Vec3Type position,
          Parameters::Vec3Type orientation,
          TrueMap &trueMap, BeliefMap &beliefMap)
            : _position(position), _orientation(orientation),
              _sensor(position, orientation),
              _trueMap(trueMap), _beliefMap(beliefMap),
              _lastTime(0), _observationMode(OBSERVE_ONLY_REAL),
              _step(0), _lastPosition(position), _travelledDistance(0)
    {
    }

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

    Parameters::NumType yaw() const
    {
        return _yaw;
    }

    void setYaw(double yaw)
    {
        _yaw = yaw;
        setOrientation(Parameters::Vec3Type(std::cos(yaw), std::sin(yaw), 0));
    }

    Observation observe()
    {
        switch (_observationMode)
        {
            case OBSERVE_ONLY_IMAGINARY:
                return _sensor.observeImaginary(_beliefMap);
            case OBSERVE_BOTH:
                if (_step < 1)
                {
//                    ROS_INFO("TAKE TRUE MEASUREMENT !!!!!!!!!!!!!!!!");
                    return _sensor.observe(_trueMap);
                }
                return _sensor.observeImaginary(_beliefMap);
            default:
                return _sensor.observe(_trueMap);
        }
    }

    SENSOR &sensor()
    {
        return _sensor;
    }

    void run()
    {
        Robot::publishObservation(observe());
    }

    void stop()
    {
        ROS_INFO("FakeRobot is stopping...");
        _stopRequested = true;
    }

    unsigned int currentStep() const
    {
        return _step;
    }

    unsigned int totalSteps() const
    {
        return _totalSteps;
    }

    double time() const
    {
        return _lastTime;
    }

    double velocity() const
    {
        return _lastVelocity;
    }

    ObservationMode observationMode() const
    {
        return _observationMode;
    }

    void setObservationMode(ObservationMode mode)
    {
        _observationMode = mode;
    }

    double futureReachability() const
    {
        return _futureReachability;
    }

    double futureArcLength() const
    {
        return _futureArcLength;
    }

private:
    Parameters::Vec3Type _position;
    Parameters::Vec3Type _orientation;
    Parameters::NumType _yaw;
    TrueMap _trueMap;
    BeliefMap &_beliefMap;

    Parameters::Vec3Type _lastPosition;
    double _travelledDistance;

    SENSOR _sensor;
    bool _stopRequested;
    unsigned int _step;
    unsigned int _totalSteps;

    double _futureReachability;
    double _futureArcLength;

    double _lastTime;
    double _lastVelocity;

    ObservationMode _observationMode;
};