#include "../include/Trajectory.h"

#include <fstream>

#include <ros/console.h>

Trajectory::Trajectory(std::initializer_list<Point> points, unsigned int degree)
    : _lastU(0), _yawWasPi(false)
{
    _spline = ts::BSpline(degree, DIMENSIONS, points.size(), TS_CLAMPED);
    std::vector<float> ctrlpts = _spline.ctrlp();
    unsigned int i = 0;
    for (auto &point : points)
    {
        ctrlpts[i++] = (float)point.x;
        ctrlpts[i++] = (float)point.y;
#if (DIMENSIONS == 3)
        ctrlpts[i++] = (float)point.z;
#endif
    }
    _spline.setCtrlp(ctrlpts);

    _totalArcLength = 0;
    _maxRad = Parameters::FakeRobotNumSteps * Parameters::FakeRobotAngularVelocity;

    if (Parameters::EquidistantArcLengthSampling)
    {
        auto lastResult = _spline.evaluate(0).result();
        _miniStep = 1. / _maxRad;
        _smallStep = 1e-2 / _maxRad;
        for (double u = _smallStep; u <= 1.; u += _smallStep)
        {
            auto result = _spline.evaluate(u).result();
            _totalArcLength += std::sqrt(std::pow(result[0] - lastResult[0], 2.f)
                                         + std::pow(result[1] - lastResult[1], 2.f));
            lastResult = result;
        }
        // * 5e3;


//        _smallStep = 1. / _maxRad;
//        _miniStep = _smallStep / 5.;
    }
    else
    {
        _smallStep = 1. / _maxRad;
        _miniStep = _smallStep;
        _totalArcLength = Parameters::FakeRobotNumSteps;
    }
}

TrajectoryEvaluationResult Trajectory::evaluate(double u, bool computeTime)
{
    _lastU = u;
    computeTime = computeTime && _planningPoints.empty() && _times.size() == _planningPoints.size();
    auto lastResult = _spline.evaluate(0).result();
    if (u <= 0)
    {
        auto r = TrajectoryEvaluationResult(Point(lastResult[0], lastResult[1]), _computeYaw(0), 0., 0.);
        if (computeTime)
        {
            r.time = 0;
            r.velocity = _planningPoints[0].velocity;
        }
        return r;
    }
    if (u >= 1)
    {
        lastResult = _spline.evaluate(1).result();
        auto r = TrajectoryEvaluationResult(Point(lastResult[0], lastResult[1]), _computeYaw(1), _totalArcLength, 1.);
        if (computeTime)
        {
            r.time = _totalTime;
            r.velocity = _planningPoints.back().velocity;
        }
        return r;
    }
    if (Parameters::EquidistantArcLengthSampling)
    {
        //ROS_INFO("Evaluating spline at %f stepsize is %f", u, _smallStep);
        double arcLength = 0;
        for (double v = _smallStep; v <= 1.; v += _smallStep)
        {
            //ROS_INFO("V = %f,  a = %f", v, arcLength);
            auto result = _spline.evaluate(v).result();
            arcLength += std::sqrt(std::pow(result[0] - lastResult[0], 2.f)
                                   + std::pow(result[1] - lastResult[1], 2.f));
            if (arcLength / _totalArcLength >= u)
            {
                //ROS_INFO("Arc length is %f", arcLength);
                auto r = TrajectoryEvaluationResult(Point(result[0], result[1]), _computeYaw(v), arcLength, v);
                if (computeTime)
                {
                    unsigned int ppi = (unsigned int)std::floor(v * _planningPoints.size());
                    // linear interpolation between planning points
                    auto left = _planningPoints[ppi];
                    auto right = _planningPoints[ppi + 1];
                    double alpha = (v - left.u) / (right.u - left.u);
                    r.time = _times[ppi] * alpha + _times[ppi + 1] * (1. - alpha);
                    r.velocity = _planningPoints[ppi].velocity * alpha + _planningPoints[ppi + 1].velocity * (1. - alpha);
                }
                return r;
            }
            lastResult = result;
        }

        //ROS_INFO("TOTAL Arc length is %f   estimated: %f", arcLength, _totalArcLength);
        auto r = TrajectoryEvaluationResult(Point(lastResult[0], lastResult[1]), _computeYaw(1), _totalArcLength, 1.);
        if (computeTime)
        {
            r.time = _totalTime;
            r.velocity = _planningPoints.back().velocity;
        }
        return r;
    }
    else
    {
        auto result = _spline.evaluate(u).result();
        auto r = TrajectoryEvaluationResult(Point(result[0], result[1]), _computeYaw(u), u * _totalArcLength, u);
        if (computeTime)
        {
            unsigned int ppi = (unsigned int)std::floor(u * _planningPoints.size());
            // linear interpolation between planning points
            auto left = _planningPoints[ppi];
            auto right = _planningPoints[ppi + 1];
            double alpha = (u - left.u) / (right.u - left.u);
            r.time = _times[ppi] * alpha + _times[ppi + 1] * (1. - alpha);
            r.velocity = _planningPoints[ppi].velocity * alpha + _planningPoints[ppi + 1].velocity * (1. - alpha);
        }
        return r;
    }
}

Parameters::KeySet Trajectory::computeSplineVoxels(TrueMap &trueMap)
{
    _splineVoxels.clear();
    for (double p = 0; p <= 1.; p += 1. / Parameters::FakeRobotNumSteps)
    {
        auto result = evaluate(p);
#if (DIMENSIONS == 3)
        _splineVoxels.insert(trueMap.coordToKey(result.point.x, result.point.y, result.point.z));
#else
        _splineVoxels.insert(trueMap.coordToKey(result.point.x, result.point.y, 0.05));
#endif
    }
    return _splineVoxels;
}

double Trajectory::_computeYaw(double u)
{
    auto next = _spline.evaluate((float) std::min(1.0, std::max(_smallStep, u + _miniStep))).result();
    auto prev = _spline.evaluate((float) std::min(1. - _smallStep, std::max(0.0, u - _miniStep))).result();
    double t = std::atan2(next[0] - prev[0], next[1] - prev[1]);
    if (t > 2.9)
        _yawWasPi = true;
    else if (t < 0. && _yawWasPi)
        t += 2. * M_PI;
    return t;
}

const std::vector<Point> Trajectory::controlPoints()
{
    std::vector<Point> points;
    for (unsigned int i = 0; i < _spline.nCtrlp(); ++i)
    {
        Point p;
        p.x = _spline.ctrlp()[i];
        p.y = _spline.ctrlp()[i + 1];
#if (DIMENSIONS == 3)
        p.z = _spline.ctrlp()[i + 2];
#endif
    }
    return points;
}

std::vector<Trajectory::VelocityPlanningPoint> Trajectory::computeVelocities(const VelocityPlanningParameters &parameters)
{
    _planningPoints.reserve(Parameters::VelocityPlanningPoints + 1);

    // arc length between equidistant planning points
    double deltaArc = _totalArcLength / Parameters::VelocityPlanningPoints;

    double maxRotationalVelocity = parameters.maxVelocity;

    // compute equidistant planning points
    for (unsigned int i = 0; i <= Parameters::VelocityPlanningPoints; ++i)
    {
        Trajectory::VelocityPlanningPoint vpp;
        vpp.u = i * 1. / Parameters::VelocityPlanningPoints;
        auto evaluation = evaluate(vpp.u);
        vpp.point = evaluation.point;
        double curvature = evaluateCurvature(vpp.u);
        vpp.maxVelocity = parameters.maxVelocity;
        vpp.curvature = curvature;
        if (curvature > 0.4)
            vpp.maxVelocity = 0;
        else if (curvature > 0)
            vpp.maxVelocity = std::min(vpp.maxVelocity, maxRotationalVelocity / (curvature * 6.));

        double leftFirstDerivative = std::abs(_computeYaw(vpp.u) - _computeYaw(vpp.u - _miniStep)) / (10. * _miniStep);
        double rightFirstDerivative = std::abs(_computeYaw(vpp.u + _miniStep) - _computeYaw(vpp.u)) / (10. * _miniStep);
        vpp.firstDerivative = 0.5 * (leftFirstDerivative + rightFirstDerivative);
        vpp.secondDerivative = std::abs(rightFirstDerivative - leftFirstDerivative) / (10. * _miniStep);
        vpp.curvature = std::abs(vpp.secondDerivative) / std::pow(1. + std::pow(vpp.firstDerivative, 2.), 3. / 2.);

        if (i == 0)
            vpp.velocity = vpp.maxVelocity = parameters.startVelocity;
        else if (i == Parameters::VelocityPlanningPoints)
            vpp.velocity = vpp.maxVelocity = parameters.endVelocity;
        else
            vpp.velocity = parameters.maxVelocity;
        _planningPoints[i] = vpp;
    }

    // update velocities (forward pass)
    double v = parameters.startVelocity;
    for (int i = 0; i <= Parameters::VelocityPlanningPoints; ++i)
    {
        auto &p = _planningPoints[i];
        p.velocity = std::min(p.maxVelocity, std::min(p.velocity, v));
        v = p.velocity + parameters.acceleration * deltaArc;
        v = std::min(v, parameters.maxVelocity);
    }

    // update velocities (backward pass)
    v = parameters.endVelocity;
    for (int i = Parameters::VelocityPlanningPoints; i >= 0; --i)
    {
        auto &p = _planningPoints[i];
        p.velocity = std::min(p.velocity, v);
        v = p.velocity + parameters.acceleration * deltaArc;
        v = std::min(v, parameters.maxVelocity);
    }

    return _planningPoints;
}

double Trajectory::evaluateCurvature(double u)
{
    double leftFirstDerivative = std::abs(_computeYaw(u) - _computeYaw(u - _miniStep)) / (10. * _miniStep);
    double rightFirstDerivative = std::abs(_computeYaw(u + _miniStep) - _computeYaw(u)) / (10. * _miniStep);
    double firstDerivative = 0.5 * (leftFirstDerivative + rightFirstDerivative);
    double secondDerivative = std::abs(rightFirstDerivative - leftFirstDerivative) / (10. * _miniStep);
    return std::abs(secondDerivative) / std::pow(1. + std::pow(firstDerivative, 2.), 1.5);
}

int called = 0;
std::valarray<double>
Trajectory::computeTimeProfile()
{
    _totalTime = 0;
    _relativeTotalTime = 0;
    _times = std::valarray<double>(Parameters::VelocityPlanningPoints + 1);
    _times[0] = 0;

    std::fstream f;
    f.open("trajectories/trajectory_" + std::to_string(++called) + ".csv", std::ios::out);

    for (unsigned int i = 1; i <= Parameters::VelocityPlanningPoints; ++i)
    {
        auto &p = _planningPoints[i];
        auto &lastp = _planningPoints[i - 1];
        double deltaArc = p.u - lastp.u;
        if (p.velocity + lastp.velocity <= 0.)
        {
            ROS_ERROR("Error: p.velocity + lastp.velocity <= 0");
            continue;
        }
        double deltaTime = (2. * deltaArc / (p.velocity + lastp.velocity));
        _totalTime += deltaTime;
        _relativeTotalTime += p.velocity;
        _times[i] = _relativeTotalTime;

        f << _computeYaw(p.u) << "," << p.firstDerivative << ","
          << p.secondDerivative << "," << p.curvature << ","
          << p.velocity << "," << p.maxVelocity << ","
          << p.point.x << "," << p.point.y
       #if (DIMENSIONS == 3)
          << "," << p.point.z
       #endif
          << std::endl;
    }

    f.close();
    _times *= _totalTime / _relativeTotalTime;
    return _times;
}