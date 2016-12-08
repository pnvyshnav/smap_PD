#pragma once

#include <vector>
#include <type_traits>

#include <ecl/time/stopwatch.hpp>

#include "tinysplinecpp.h"

#include "Parameters.hpp"
#include "StereoCameraSensor.h"
#include "Observation.hpp"
#include "TrueMap.h"
#include "Robot.hpp"
#include "LogOddsMap.h"


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
        _step = 0;
        ecl::StopWatch stopWatch;

        std::vector<Parameters::Vec3Type> positions = std::vector<Parameters::Vec3Type> {
                Parameters::Vec3Type( 0.05,  0.95, 0.05),
                Parameters::Vec3Type(-0.25,  0.05, 0.05),
                Parameters::Vec3Type(-0.95, -0.25, 0.05)
        };
        double maxRad = Parameters::FakeRobotNumSteps * Parameters::FakeRobotAngularVelocity;
        for (auto rad = 0.;
             !_stopRequested && _step <= Parameters::FakeRobotNumSteps;
             rad += Parameters::FakeRobotAngularVelocity, ++_step)
        {
#ifdef FAKE_2D
    #ifdef PLANNER_2D_TEST
            auto spline = _splines[_splineId];
            float overallProgress = rad / maxRad;
            std::vector<float> result = spline.evaluate(overallProgress).result();
            setPosition(Parameters::Vec3Type(result[0], result[1], 0.05));

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

            float miniStep = 0.5f / maxRad;
            auto next = spline.evaluate(std::min(1.0f, overallProgress+miniStep)).result();
            auto prev = spline.evaluate(std::max(0.0f, overallProgress-miniStep)).result();
            double angle = std::atan2(next[0] - prev[0], next[1] - prev[1]);
            setOrientation(Parameters::Vec3Type(std::sin(angle), std::cos(angle), 0));
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

    /**
     * Computes the reachability of the current active trajectory at the current step.
     * @return Reachability of voxels along the current trajectory.
     */
    double currentReachability(LogOddsMap &map) const
    {
        unsigned int step = 0;
        double maxRad = Parameters::FakeRobotNumSteps * Parameters::FakeRobotAngularVelocity;
        for (auto rad = 0.; step <= Parameters::FakeRobotNumSteps;
             rad += Parameters::FakeRobotAngularVelocity, ++step)
        {

        }
    }

    void selectSpline(unsigned int splineId)
    {
        _splineId = splineId;
    }

private:
    Parameters::Vec3Type _position;
    Parameters::Vec3Type _orientation;
    TrueMap _trueMap;
    SENSOR _sensor;
    bool _stopRequested;
    unsigned int _step;
    std::vector<ts::BSpline> _splines;
    unsigned int _splineId;
};