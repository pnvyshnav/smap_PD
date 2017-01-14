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

struct SplineEvaluationResult
{
    double x;
    double y;
    double arcLength;
    double u;
    SplineEvaluationResult(double x, double y, double arcLength, double u)
            : x(x), y(y), arcLength(arcLength), u(u)
    {}
};


#define EQUIDISTANT_ARC_LENGTH


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
          TrueMap &trueMap)
            : _position(position), _orientation(orientation),
              _sensor(position, orientation),
              _trueMap(trueMap),
              _splineId(0)
    {
        // Trajectories to evaluate
        ts::BSpline spline1(1, 2, 3, TS_CLAMPED);
        std::vector<float> ctrlp1 = spline1.ctrlp();
        ctrlp1[0]  =  0.0f;
        ctrlp1[1]  = -0.9f;

        ctrlp1[2]  =  0.0f;
        ctrlp1[3]  =  0.0f;

        ctrlp1[4]  = -0.9f;
        ctrlp1[5]  =  0.0f;
        spline1.setCtrlp(ctrlp1);
        _splines.push_back(spline1);

        ts::BSpline spline2(2, 2, 4, TS_CLAMPED);
        std::vector<float> ctrlp2 = spline2.ctrlp();
        ctrlp2[0]  =  0.0f;
        ctrlp2[1]  = -0.9f;

        ctrlp2[2]  =  0.5f;
        ctrlp2[3]  = -0.1f;

        ctrlp2[4]  =  0.0f;
        ctrlp2[5]  =  0.0f;

        ctrlp2[6]  = -0.9f;
        ctrlp2[7]  =  0.0f;
        spline2.setCtrlp(ctrlp2);
        _splines.push_back(spline2);

        ts::BSpline spline3(2, 2, 4, TS_CLAMPED);
        std::vector<float> ctrlp3 = spline3.ctrlp();
        ctrlp3[0]  =  0.0f;
        ctrlp3[1]  = -0.9f;

        ctrlp3[2]  =  0.0f;
        ctrlp3[3]  =  0.4f;

        ctrlp3[4]  =  0.4f;
        ctrlp3[5]  =  0.0f;

        ctrlp3[6]  = -0.9f;
        ctrlp3[7]  =  0.0f;
        spline3.setCtrlp(ctrlp3);
        _splines.push_back(spline3);
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
        return _sensor.observe(_trueMap);
    }

    SENSOR &sensor()
    {
        return _sensor;
    }

    /**
     * Evaluates the selected spline at position u (u in [0,1]).
     * @param u Relative position on spline in [0,1].
     * @param splineId Id of spline to evaluate.
     * @return Point on the spline.
     */
    SplineEvaluationResult evaluateSpline(double u, unsigned int splineId) const
    {
        auto lastResult = _splines[splineId].evaluate(0).result();
        if (u <= 0)
            return SplineEvaluationResult(lastResult[0], lastResult[1], 0., 0.);
        if (u >= 1)
        {
            lastResult = _splines[splineId].evaluate(1).result();
            return SplineEvaluationResult(lastResult[0], lastResult[1], _currentSplineArcLength, 1.);
        }
#ifdef EQUIDISTANT_ARC_LENGTH
        //ROS_INFO("Evaluating spline at %f  stepsize is %f", u, _splineSmallStepSize);
        double arcLength = 0;
        for (double v = _splineSmallStepSize; v <= 1.; v += _splineSmallStepSize)
        {
            //ROS_INFO("V = %f,  a = %f", v, arcLength);
            auto result = _splines[splineId].evaluate(v).result();
            arcLength += std::sqrt(std::pow(result[0]-lastResult[0], 2.f)
                                   + std::pow(result[1]-lastResult[1], 2.f));
            if (arcLength / _currentSplineArcLength >= u)
            {
                //ROS_INFO("Arc length is %f", arcLength);
                return SplineEvaluationResult(result[0], result[1], arcLength, v);
            }
            lastResult = result;
        }
        //ROS_INFO("TOTAL Arc length is %f   estimated: %f", arcLength, _currentSplineArcLength);

        return SplineEvaluationResult(lastResult[0], lastResult[1], _currentSplineArcLength, 1.);
#else
        auto result = _splines[splineId].evaluate(u).result();
        return SplineEvaluationResult(result[0], result[1], u * _currentSplineArcLength, u);
#endif
    }

    /**
     * Evaluates the current spline at position u (u in [0,1]).
     * @param u Relative position on spline in [0,1].
     * @return Point on the spline.
     */
    SplineEvaluationResult evaluateSpline(double u) const
    {
        return evaluateSpline(u, _splineId);
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
        if (!_currentSplineTiming.empty())
        {
            stepLimit = _currentSplineTiming.size();
        }

        double maxRad = stepLimit * Parameters::FakeRobotAngularVelocity;

        for (auto rad = 0.;
             !_stopRequested && _step <= stepLimit;
             rad += Parameters::FakeRobotAngularVelocity, ++_step)
        {
#ifdef FAKE_2D
    #ifdef PLANNER_2D_TEST
            auto spline = _splines[_splineId];
            double overallProgress = rad / maxRad;
            auto current = evaluateSpline(overallProgress);

            double miniStep = _splineSmallStepSize / 5.;
    #ifdef EQUIDISTANT_ARC_LENGTH
            miniStep *= 1e3;
    #endif
            auto next = spline.evaluate((float)std::min(1.0, current.u + miniStep)).result();
            auto prev = spline.evaluate((float)std::min(0.99999, std::max(0.0, current.u - miniStep))).result();
            _yaw = std::atan2(next[0] - prev[0], next[1] - prev[1]);

            if (_splineId == 0)
            {
                if (overallProgress < 0.5)
                    _yaw = 0;
                else
                    _yaw = -M_PI / 2.;
            }

            setPosition(Parameters::Vec3Type(current.x, current.y, 0.05));

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

            if (!_currentSplineTiming.empty())
            {
                if (_step >= _currentSplineTiming.size())
                    break;

                double time = _currentSplineTiming[_step];
                ROS_INFO("Time: %f", time);

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
                    double p = futureStep * Parameters::FakeRobotAngularVelocity / maxRad;
                    auto result = evaluateSpline(p);
                    //_splineFutureVoxels.insert(_trueMap.coordToKey(result[0], result[1], 0.05));

                    // sample from the environment
                    for (int x = -1; x <= 1; ++x)
                    {
                        for (int y = -1; y <= 1; ++y)
                        {
                            _splineFutureVoxels.insert(
                                    _trueMap.coordToKey(result.x + x * 0.5 * Parameters::voxelSize,
                                                        result.y + y * 0.5 * Parameters::voxelSize,
                                                        0.05));
                        }
                    }

                    double futureTime = _currentSplineTiming[futureStep];
                    elapsedFuture += 1. / (futureTime - lastFuture);
                    if (elapsedFuture > 1.5) //Parameters::EvaluateFutureTimespan)
                        break;
                    lastFuture = futureTime;
                }
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

    std::vector<ts::BSpline> splines() const
    {
        return _splines;
    }

    unsigned int selectedSpline() const
    {
        return _splineId;
    }

    double time() const
    {
        if (_currentSplineTiming.empty())
            return -1;
        if (_step < _currentSplineTiming.size())
            return _currentSplineTiming[_step];
        return _currentSplineTiming.back();
    }

    void selectSpline(unsigned int splineId)
    {
        _splineId = splineId;

#ifdef PLANNER_2D_TEST
        double maxRad = Parameters::FakeRobotNumSteps * Parameters::FakeRobotAngularVelocity;

        _currentSplineArcLength = 0;
        auto lastResult = _splines[_splineId].evaluate(0).result();
#ifdef EQUIDISTANT_ARC_LENGTH
        _splineSmallStepSize = 1e-3 / maxRad;
#else
        _splineSmallStepSize = 1. / maxRad;
#endif
        for (double u = _splineSmallStepSize; u <= 1.; u += _splineSmallStepSize)
        {
            auto result = _splines[_splineId].evaluate(u).result();
            _currentSplineArcLength += std::sqrt(std::pow(result[0]-lastResult[0], 2.f)
                                        + std::pow(result[1]-lastResult[1], 2.f));
            lastResult = result;
        }
        ROS_INFO("Arc Length of Current Spline: %f", _currentSplineArcLength);

        auto spline = _splines[_splineId];

        // update spline voxels
        _splineVoxels.clear();
        for (double p = 0; p <= 1.; p += 1. / Parameters::FakeRobotNumSteps)
        {
            auto result = evaluateSpline(p);
            _splineVoxels.insert(_trueMap.coordToKey(result.x, result.y, 0.05));
        }
        ROS_INFO("Selected spline %d with %d voxels.", (int)splineId, (int)_splineVoxels.size());

        // load timing info if available
        std::ifstream is("/home/eric/catkin_ws/src/smap/stats/timing_" + std::to_string(_splineId) + ".csv");
        if (!is.bad())
        {
            std::istream_iterator<double> start(is), end;
            _currentSplineTiming = std::vector<double>(start, end);
            ROS_INFO("Successfully loaded timings for spline %d with %d times.", (int)splineId, (int)_currentSplineTiming.size());
        }
#endif
    }

    /**
     * Returns the keys to the voxels covered by the current spline.
     * @return Set of distinct OcTreeKeys.
     */
    const Parameters::KeySet &currentSplineVoxels() const
    {
        return _splineVoxels;
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
    SENSOR _sensor;
    bool _stopRequested;
    unsigned int _step;
    std::vector<ts::BSpline> _splines;
    unsigned int _splineId;
    Parameters::KeySet _splineVoxels;
    Parameters::KeySet _splineFutureVoxels;
    std::vector<double> _currentSplineTiming;
    double _currentSplineArcLength;
    double _splineSmallStepSize;
};