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
#include "../trajopt/BSplineTrajectory.h"
#include "../trajopt/MinSnapTrajectory.h"

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
    // start, end, currentVelocity
    // TODO allow other types of trajectories
    typedef std::function<MinSnapTrajectory(Point, Point, Point)> PlanningSubscriber;

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
//        ROS_INFO("FakeRobot is running...");
        _stopRequested = false;
//#ifndef REPLANNING
        _step = 0;
//#endif
        ecl::StopWatch stopWatch;

        std::vector<Parameters::Vec3Type> positions = std::vector<Parameters::Vec3Type> {
                Parameters::Vec3Type( 0.05,  0.95, 0.05),
                Parameters::Vec3Type(-0.25,  0.05, 0.05),
                Parameters::Vec3Type(-0.95, -0.25, 0.05)
        };

        double lastTime = 0;
        unsigned int stepLimit;
#ifdef SIMULATE_TIME
        stepLimit = (unsigned int)(Parameters::SimulationFinalTime / Parameters::SimulationTimeStep);
#else
        _totalSteps = Parameters::FakeRobotNumSteps;
//        if (_observationMode == OBSERVE_ONLY_REAL)
            _totalSteps *= 2;
#endif

        for (double rad = 0.;
             !_stopRequested && _step < _totalSteps;
             rad += Parameters::FakeRobotAngularVelocity, ++_step)
        {
#ifdef FAKE_2D
    #ifdef PLANNER_2D_TEST
            //ROS_INFO("BSplineTrajectory empty? %d", (int) _trajectory.empty());
//            if (!_trajectory->empty())
//            {
                TrajectoryEvaluationResult current;
    #ifdef SIMULATE_TIME
                double time = _step * Parameters::SimulationTimeStep;
                //ROS_INFO("Computing trajectory at time = %f", time);
                current = _trajectory->evaluateAtTime(time);
    #else
                double u = _step * 1. / _totalSteps;
                //ROS_INFO("Computing trajectory at u = %f", u);
                current = _trajectory.evaluate(u, true);
    #endif

                _yaw = current.yaw;
                _lastVelocity = current.velocity;

                setPosition(Parameters::Vec3Type(current.point.x, current.point.y, 0.05));

            if (_observationMode == OBSERVE_ONLY_REAL)
            {
                if (_step == 0)
                    _lastPosition = _position;
                _travelledDistance += _position.distance(_lastPosition);
//                std::cout << "Distance from " << _position << " to " << _lastPosition << std::endl << _position.distance(_lastPosition) << std::endl;
            }

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

                if (current.u >= 1. || current.time >= _trajectory.totalTime())
                    break;


                Robot::publishObservation(observe());
//            _beliefMap.publish();

                //ROS_INFO("Current Robot Time: %f", time);

    #ifdef SIMULATE_TIME
//                ros::Rate publishing_rate((time - lastTime) + 2);
//                publishing_rate.sleep();
//                lastTime = time;
    #endif

                _splineFutureVoxels.clear();

            std::vector<Point> futurePositions;

                for (unsigned int futureStep = _step + 1;
                     futureStep < std::min(_totalSteps, _step + 17); // TODO change back to + 11
                     ++futureStep)
                {
                    TrajectoryEvaluationResult fresult;
    #ifdef SIMULATE_TIME
                    double ftime = futureStep * Parameters::SimulationTimeStep;
                    //ROS_INFO("Computing trajectory at time = %f", time);
                    fresult = _trajectory->evaluateAtTime(ftime);
    #else
                    double p = futureStep * 1. / _totalSteps;
                    fresult = _trajectory.evaluate(p, true);
    #endif
                    futurePositions.push_back(fresult.point);
                    octomap::OcTreeKey key = _trueMap.coordToKey(fresult.point.x, fresult.point.y, 0.05);
                    _splineFutureVoxels.insert(key);

                    // sample from the environment // todo makes sense for stereo camera with multiple pixels
//                    for (int x = -1; x <= 1; ++x)
//                    {
//                        for (int y = -1; y <= 1; ++y)
//                        {
//                            _splineFutureVoxels.insert(
//                                    _trueMap.coordToKey(fresult.point.x + x * 0.5 * Parameters::voxelSize,
//                                                        fresult.point.y + y * 0.5 * Parameters::voxelSize,
//                                                        0.05));
//                        }
//                    }

//                    double elapsedFuture = fresult.time - current.time;
//                    if (elapsedFuture > 0.1 * Parameters::SimulationTimeStep) //Parameters::EvaluateFutureTimespan)
//                        break;
                }
#ifdef SIMULATE_TIME
                _lastTime = time;
                _lastVelocity = current.velocity;
#endif

        #if defined(REPLANNING)
                    _futureReachability = 1;
                    Point lastPosition = current.point;
                    _futureArcLength = 0;
//                    ROS_INFO("Checking future _futureReachability...");
                    // TODO check if replanning is necessary
                    for (auto &p : futurePositions)
                    {
                        _futureReachability *= std::pow(_beliefMap.filteredReachability(p.x, p.y, 0.05),
                                                        p.dist(lastPosition));
                        lastPosition = p;
                        _futureArcLength += p.dist(lastPosition);
                    }
//                    if (futurePositions.size() > 1)
//                        reachability = std::pow(reachability, 1. / _futureArcLength);
//                    for (auto &voxel : _splineFutureVoxels)
//                    {
//                        auto beliefVoxel = _beliefMap.query(voxel);
//                        if (!beliefVoxel.node())
//                        {
//                            ROS_WARN("Could not evaluate future belief voxel");
//                            continue;
//                        }
//                        reachability *= 1. - _beliefMap.getVoxelMean(beliefVoxel);
//                    }
                if (_observationMode == OBSERVE_ONLY_REAL)
                {
                    ROS_INFO("Future Reachability: %f  (%i voxels, %i positions)",
                             _futureReachability, (int)_splineFutureVoxels.size(), (int)futurePositions.size());
                    ROS_INFO("Travelled Distance: %f", _travelledDistance);
                    auto replanningNecessary = _futureReachability < 0.9;
                    if (_replanningHandler && (replanningNecessary)) // || _travelledDistance > 10 * Parameters::voxelSize))
                    {
                        if (!replanningNecessary && _trajectory.end().dist(current.point) < 1.)
                        {
                            ROS_INFO("Almost there (%f), no replanning necessary",
                                     _trajectory.end().dist(current.point));
                        }
                        else if (_travelledDistance < 1e-3)
                        {
                            ROS_INFO("Did not travel enough distance to replan (%f).", _travelledDistance);
                        }
                        else
                        {
                            ROS_INFO("REPLANNING IS NECESSARY (reachability: %f)", _futureReachability);
                            // TODO fix that trajectory/robot "forgets" the end point
//                        setTrajectory(_replanningHandler(current.point, _trajectory.end(), current.velocity));
                            auto velocity = ((MinSnapTrajectory)_trajectory).velocity(u);
                            ROS_INFO("Velocity at replanning: %f %f", velocity.x, velocity.y);
                            auto trajectory = _replanningHandler(Point(_position.x(), _position.y()),
                                                                 _trajectory.end(), velocity);
                            setTrajectory(trajectory);
                            _step = 0; // step has to be reset manually
                            run();
                            _lastPosition = _position;
                            _travelledDistance = 0;
                            return;
                        }
                    }
                }
        #endif
//            }
    #else
            setOrientation(Parameters::Vec3Type(std::cos(rad), std::sin(rad), 0));
            Robot::publishObservation(observe());
    #endif
#else
            const static Parameters::NumType radius = 0.7;
            setOrientation(Parameters::Vec3Type(std::sin(-rad), std::cos(rad), 0));
            // let the orientation "lag behind" the position to see more from the side
//            setPosition(Parameters::Vec3Type(
//                    radius * std::cos(rad + 1.5*Parameters::FakeRobotAngularVelocity),
//                    radius * std::sin(rad + 1.5*Parameters::FakeRobotAngularVelocity), 0));

            Robot::publishObservation(observe());
#endif

//#if defined(FAKE_3D) || defined(FAKE_2D)
//            if (_step % 20 == 0)
//                ROS_INFO("Robot step %i / %i", (int)_step+1, (int)Parameters::FakeRobotNumSteps);
//#endif
        }

//#if defined(FAKE_3D) || defined(FAKE_2D)
//        if (_stopRequested)
//            ROS_INFO("Robot stopped after %g full rounds in %d steps.",
//                     (Parameters::FakeRobotNumSteps * Parameters::FakeRobotAngularVelocity)/(2. * M_PI),
//                     (int)_step);
//        else
//            ROS_INFO("Robot completed %g full rounds in %d steps.",
//                     (Parameters::FakeRobotNumSteps * Parameters::FakeRobotAngularVelocity)/(2. * M_PI),
//                     (int)_step);
//
//        ROS_INFO_STREAM("Elapsed time: " << stopWatch.elapsed() << " seconds");
//#endif
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

    MinSnapTrajectory trajectory()
    {
        return _trajectory;
    }

    void setTrajectory(MinSnapTrajectory trajectory)
    {
        _trajectory = trajectory;
        _travelledDistance = 0;
    }

    void setReplanningHandler(PlanningSubscriber handler)
    {
        _replanningHandler = handler;
    }

    ObservationMode observationMode() const
    {
        return _observationMode;
    }

    void setObservationMode(ObservationMode mode)
    {
        _observationMode = mode;
    }

    /**
     * Returns the keys to the voxels covered within the next Parameters::EvaluateFutureSteps by the current spline.
     * @return Set of distinct OcTreeKeys.
     */
    const Parameters::KeySet &currentSplineFutureVoxels() const
    {
        return _splineFutureVoxels;
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
    Parameters::KeySet _splineFutureVoxels;

    double _futureReachability;
    double _futureArcLength;

    MinSnapTrajectory _trajectory;
    double _lastTime;
    double _lastVelocity;

    PlanningSubscriber _replanningHandler;

    ObservationMode _observationMode;
};