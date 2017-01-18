#pragma once

#include <vector>
#include <type_traits>
#include <unordered_set>

#include <ecl/time/stopwatch.hpp>

#include "tinysplinecpp.h"

#include "Parameters.hpp"
#include "StereoCameraSensor.h"
#include "Observation.hpp"
#include "TrueMap.h"
#include "Robot.hpp"
#include "LogOddsMap.h"
#include "Trajectory.h"


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
              _lastTime(0)
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

    Observation observe()
    {
        if (_step < 5)
            return _sensor.observe(_trueMap);
        return _sensor.observeImaginary(_beliefMap);
    }

    SENSOR &sensor()
    {
        return _sensor;
    }

    void run()
    {
        ROS_INFO("FakeRobot is running...");
        _stopRequested = false;
        _step = 0;
        ecl::StopWatch stopWatch;

        std::vector<Parameters::Vec3Type> positions = std::vector<Parameters::Vec3Type> {
                Parameters::Vec3Type( 0.05,  0.95, 0.05),
                Parameters::Vec3Type(-0.25,  0.05, 0.05),
                Parameters::Vec3Type(-0.95, -0.25, 0.05)
        };

        double lastTime = 0;
        unsigned int stepLimit = Parameters::FakeRobotNumSteps;

        double maxRad = stepLimit * Parameters::FakeRobotAngularVelocity;

        for (auto rad = 0.;
             !_stopRequested && _step <= stepLimit;
             rad += Parameters::FakeRobotAngularVelocity, ++_step)
        {
#ifdef FAKE_2D
    #ifdef PLANNER_2D_TEST
            //ROS_INFO("Trajectory empty? %d", (int) _trajectory.empty());
            if (!_trajectory.empty())
            {
                double u = _step * 1. / stepLimit;
                //ROS_INFO("Computing trajectory at u = %f", u);
                auto current = _trajectory.evaluate(u, true);

                _yaw = current.yaw;

                setPosition(Parameters::Vec3Type(current.point.x, current.point.y, 0.05));

    //            int pos = (int)(rad / (2. * M_PI));
    //            double progress = rad / (2. * M_PI) - pos;
    //            auto next = pos < positions.size()-1 ? positions[pos+1] : positions[pos];
    //            if (pos >= positions.size())
    //                pos = positions.size()-1;
    //            ROS_INFO("pos: %d", pos);
    //            ROS_INFO("progress: %f", progress);
    //            setPosition(positions[pos] + (next - positions[pos]) * progress);

    //            double angle = std::atan2(next.x()-positions[pos].x(), next.y()-positions[pos].y()) + M_PI_4;
    //            setOrientation(Parameters::Vec3Type(-std::cos(angle), -std::sin(angle), 0));

                setOrientation(Parameters::Vec3Type(std::sin(_yaw), std::cos(_yaw), 0));

                if (u >= 1.)
                    break;

                double time = current.time;
                //ROS_INFO("Current Robot Time: %f", time);

    #ifdef SIMULATE_TIME
                ros::Rate publishing_rate((time - lastTime) + 2);
                publishing_rate.sleep();
                lastTime = time;
    #endif

                _splineFutureVoxels.clear();
                double elapsedFuture = 0, lastFuture = time;
                for (unsigned int futureStep = _step + 1;
                     futureStep < stepLimit;
                     ++futureStep)
                {
                    double p = futureStep * 1. / stepLimit;
                    auto fresult = _trajectory.evaluate(p, true);
                    //_splineFutureVoxels.insert(_trueMap.coordToKey(fresult[0], fresult[1], 0.05));

                    // sample from the environment
                    for (int x = -1; x <= 1; ++x)
                    {
                        for (int y = -1; y <= 1; ++y)
                        {
                            _splineFutureVoxels.insert(
                                    _trueMap.coordToKey(fresult.point.x + x * 0.5 * Parameters::voxelSize,
                                                        fresult.point.y + y * 0.5 * Parameters::voxelSize,
                                                        0.05));
                        }
                    }

                    double futureTime = fresult.time;
                    elapsedFuture += futureTime - lastFuture;
                    if (elapsedFuture > 0.3) //Parameters::EvaluateFutureTimespan)
                        break;
                    lastFuture = futureTime;
                }

                _lastTime = time;
            }
    #else
            setOrientation(Parameters::Vec3Type(std::cos(rad), std::sin(rad), 0));
    #endif
#else
            const static Parameters::NumType radius = 0.7;
            setOrientation(Parameters::Vec3Type(-std::sin(-rad), -std::cos(rad), 0));
            // let the orientation "lag behind" the position to see more from the side
            setPosition(Parameters::Vec3Type(
                    radius * std::cos(rad + 1.5*Parameters::FakeRobotAngularVelocity),
                    radius * std::sin(rad + 1.5*Parameters::FakeRobotAngularVelocity), 0));
#endif
            Robot::publishObservation(observe());

            if (_step % 20 == 0)
                ROS_INFO("Robot step %i / %i", (int)_step+1, (int)Parameters::FakeRobotNumSteps);
        }

        if (_stopRequested)
            ROS_INFO("Robot stopped after %g full rounds in %d steps.",
                     (Parameters::FakeRobotNumSteps * Parameters::FakeRobotAngularVelocity)/(2. * M_PI),
                     (int)_step);
        else
            ROS_INFO("Robot completed %g full rounds in %d steps.",
                     (Parameters::FakeRobotNumSteps * Parameters::FakeRobotAngularVelocity)/(2. * M_PI),
                     (int)_step);

        ROS_INFO_STREAM("Elapsed time: " << stopWatch.elapsed() << " seconds");
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

    double time() const
    {
        return _lastTime;
    }

    Trajectory &trajectory()
    {
        return _trajectory;
    }

    void setTrajectory(const Trajectory &trajectory)
    {
        _trajectory = trajectory;
    }

    /**
     * Returns the keys to the voxels covered within the next Parameters::EvaluateFutureSteps by the current spline.
     * @return Set of distinct OcTreeKeys.
     */
    const Parameters::KeySet &currentSplineFutureVoxels() const
    {
        return _splineFutureVoxels;
    }

private:
    Parameters::Vec3Type _position;
    Parameters::Vec3Type _orientation;
    Parameters::NumType _yaw;
    TrueMap _trueMap;
    BeliefMap _beliefMap;

    SENSOR _sensor;
    bool _stopRequested;
    unsigned int _step;
    Parameters::KeySet _splineFutureVoxels;

    Trajectory _trajectory;
    double _lastTime;
};