#pragma once

#include <vector>
#include <type_traits>

#include <ecl/time/stopwatch.hpp>

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
        ROS_INFO("FakeRobot is running...");
        _stopRequested = false;
        unsigned int step = 0;
        ecl::StopWatch stopWatch;
        for (auto rad = 0.;
             !_stopRequested && step < Parameters::FakeRobotNumSteps;
             rad += Parameters::FakeRobotAngularVelocity, ++step)
        {
            setOrientation(Parameters::Vec3Type(std::cos(rad), std::sin(rad), 0));
            Robot::publishObservation(observe());
        }

        if (_stopRequested)
            ROS_INFO("Robot stopped after %g full rounds in %d steps.",
                     (Parameters::FakeRobotNumSteps * Parameters::FakeRobotAngularVelocity)/(2. * M_PI),
                     (int)step);
        else
            ROS_INFO("Robot completed %g full rounds in %d steps.",
                     (Parameters::FakeRobotNumSteps * Parameters::FakeRobotAngularVelocity)/(2. * M_PI),
                     (int)step);

        ROS_INFO_STREAM("Elapsed time: " << stopWatch.elapsed() << " seconds");
    }

    void stop()
    {
        ROS_INFO("FakeRobot is stopping...");
        _stopRequested = true;
    }

private:
    Parameters::Vec3Type _position;
    Parameters::Vec3Type _orientation;
    TrueMap _trueMap;
    SENSOR _sensor;
    bool _stopRequested;
};