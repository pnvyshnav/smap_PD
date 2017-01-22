#include "../include/Trajectory.h"

#include <fstream>

#include <ros/console.h>

Trajectory::Trajectory(std::initializer_list<Point> points, unsigned int degree)
    : _lastU(0), _yawWasPi(false), _empty(false)
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
        _miniStep = 0.5 / _maxRad;
        _smallStep = 1e-2 / _maxRad;
        for (double u = _smallStep; u <= 1.; u += _smallStep)
        {
            auto result = _spline.evaluate(u).result();
            _totalArcLength += std::sqrt(std::pow(result[0] - lastResult[0], 2.f)
                                         + std::pow(result[1] - lastResult[1], 2.f));
            lastResult = result;
        }


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

Trajectory::Trajectory() : _lastU(0), _yawWasPi(false), _empty(true),
                           _totalArcLength(0), _totalTime(0), _relativeTotalTime(0)
{

}

Trajectory::Trajectory(const Trajectory &trajectory)
        : _lastU(trajectory._lastU), _yawWasPi(trajectory._yawWasPi),
          _empty(trajectory._empty), _totalArcLength(trajectory._totalArcLength),
          _totalTime(trajectory._totalTime), _relativeTotalTime(trajectory._relativeTotalTime),
          _spline(trajectory._spline),  _maxRad(trajectory._maxRad),
          _smallStep(trajectory._smallStep), _miniStep(trajectory._miniStep),
          _planningPoints(trajectory._planningPoints), _times(trajectory._times),
          _splineVoxels(trajectory._splineVoxels), _splineFutureVoxels(trajectory._splineFutureVoxels)
{

}

TrajectoryEvaluationResult Trajectory::evaluate(double u, bool computeTime)
{
    //ROS_INFO("Evaluating at u = %f with computeTime? %d", u, (int)computeTime);
    _lastU = u;
    //ROS_INFO("Planning points: %d    Times: %d", (int)_planningPoints.size(), (int)_times.size());
    computeTime = computeTime && !_planningPoints.empty() && _times.size() == _planningPoints.size();
    auto lastResult = _spline.evaluate(0).result();
    if (u <= 0)
    {
        auto r = TrajectoryEvaluationResult(Point(lastResult[0], lastResult[1]),
                                            _computeYaw(0), 0., 0., 0.,
                                            _planningPoints.front().velocity);
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
        auto r = TrajectoryEvaluationResult(Point(lastResult[0], lastResult[1]),
                                            _computeYaw(1), _totalArcLength, 1., 1.,
                                            _planningPoints.back().velocity);
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
            //ROS_INFO("V = %f,  a = %f,   u = %f,    rel.arc = %f", v, arcLength, u, arcLength / _totalArcLength);
            auto result = _spline.evaluate(v).result();
            arcLength += std::sqrt(std::pow(result[0] - lastResult[0], 2.f)
                                   + std::pow(result[1] - lastResult[1], 2.f));
            if (arcLength / _totalArcLength >= u)
            {
//                ROS_INFO("Arc length is %f", arcLength);
                auto r = TrajectoryEvaluationResult(Point(result[0], result[1]),
                                                    _computeYaw(v), arcLength,
                                                    arcLength / _totalArcLength, v, 0);
                if (computeTime)
                {
                    unsigned int ppi = (unsigned int)std::floor(v * _planningPoints.size());
                    // linear interpolation between planning points
                    auto left = _planningPoints[ppi];
                    auto right = _planningPoints[ppi + 1];
                    double alpha = (v - left.u) / (right.u - left.u);
                    r.time = _times[ppi + 1] * alpha + _times[ppi] * (1. - alpha);
                    //ROS_INFO("CTIME: %f", r.time);
                    r.velocity = _planningPoints[ppi].velocity * alpha + _planningPoints[ppi + 1].velocity * (1. - alpha);
                }
                return r;
            }
            lastResult = result;
        }

        //ROS_INFO("TOTAL Arc length is %f   estimated: %f", arcLength, _totalArcLength);
        auto r = TrajectoryEvaluationResult(Point(lastResult[0], lastResult[1]),
                                            _computeYaw(1), _totalArcLength, 1., 1.,
                                            _planningPoints.back().velocity);
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
        auto r = TrajectoryEvaluationResult(Point(result[0], result[1]), _computeYaw(u), u * _totalArcLength, u, u, 0);
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

TrajectoryEvaluationResult Trajectory::evaluateAtTime(double time)
{
    if (time <= 0.)
        return evaluate(0, true);

    for (unsigned int i = 0; i < Parameters::VelocityPlanningPoints; ++i)
    {
        auto &current = _planningPoints[i];
        auto &next = _planningPoints[i + 1];
        if (time >= current.time && time <= next.time)
        {
            double alpha = (time - current.time) / (next.time - current.time);
            return evaluate(current.u * alpha + next.u * (1. - alpha));
        }
    }
    return evaluate(1, true);
}

Parameters::KeySet Trajectory::splineVoxelKeys(const TrueMap &trueMap)
{
    //ROS_INFO("Spline voxels before computation: %d", (int)_splineVoxels.size());
    if (!_splineVoxels.empty())
        return _splineVoxels;

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

Parameters::PositionSet Trajectory::splineVoxelPositions(const TrueMap &trueMap)
{
    //ROS_INFO("Spline voxels before computation: %d", (int)_splineVoxels.size());
    if (!_splineVoxelPositions.empty())
        return _splineVoxelPositions;

    for (double p = 0; p <= 1.; p += 1. / Parameters::FakeRobotNumSteps)
    {
        auto result = evaluate(p);
#if (DIMENSIONS == 3)
        _splineVoxelPositions.insert(trueMap.keyToCoord(trueMap.coordToKey(result.point.x, result.point.y, result.point.z)));
#else
        _splineVoxelPositions.insert(trueMap.keyToCoord(trueMap.coordToKey(result.point.x, result.point.y, 0.05)));
#endif
    }
    return _splineVoxelPositions;
}

double Trajectory::_computeYaw(double u)
{
    auto next = _spline.evaluate((float) std::min(1., std::max(_miniStep/1., u + _miniStep/1.))).result();
    auto prev = _spline.evaluate((float) std::min(1. - _miniStep/1., std::max(0.0, u - _miniStep/1.))).result();
    double t = std::atan2(next[0] - prev[0], next[1] - prev[1]);
    if (t > 2.9)
        _yawWasPi = true;
    else if (t < 0. && _yawWasPi)
        t += 2. * M_PI;
    return t;
}

const std::vector<Point> Trajectory::controlPoints() const
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
        points.push_back(p);
    }
    return points;
}

std::vector<Trajectory::VelocityPlanningPoint> Trajectory::computeVelocities(
        const VelocityPlanningParameters &parameters)
{
    _planningPoints.resize(Parameters::VelocityPlanningPoints + 1);

    // arc length between equidistant planning points
    double deltaArc = _totalArcLength / Parameters::VelocityPlanningPoints;

    // compute equidistant planning points
    for (unsigned int i = 0; i <= Parameters::VelocityPlanningPoints; ++i)
    {
        Trajectory::VelocityPlanningPoint vpp;
        vpp.u = i * 1. / Parameters::VelocityPlanningPoints;
        auto evaluation = evaluate(vpp.u);
        vpp.splineU = evaluation.splineU;
        vpp.point = evaluation.point;
        double curvature = _computeCurvature(evaluation.splineU, vpp);
        vpp.maxVelocity = parameters.maxVelocity;
        if (curvature > 0.4)
            vpp.maxVelocity = 0;
        else if (curvature > 0)
            vpp.maxVelocity = std::min(vpp.maxVelocity, parameters.maxRotationalVelocity / curvature);
        vpp.maxVelocity = std::max(parameters.minVelocity, vpp.maxVelocity);

//        double leftFirstDerivative = std::abs(_computeYaw(vpp.u) - _computeYaw(vpp.u - _miniStep)) / (10. * _miniStep);
//        double rightFirstDerivative = std::abs(_computeYaw(vpp.u + _miniStep) - _computeYaw(vpp.u)) / (10. * _miniStep);
//        vpp.firstDerivative = 0.5 * (leftFirstDerivative + rightFirstDerivative);
//        vpp.secondDerivative = std::abs(rightFirstDerivative - leftFirstDerivative) / (10. * _miniStep);
//        vpp.curvature = std::abs(vpp.secondDerivative) / std::pow(1. + std::pow(vpp.firstDerivative, 2.), 3. / 2.);

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

double Trajectory::_computeCurvature(double u, VelocityPlanningPoint &vpp)
{
    double leftFirstDerivative = std::abs(_computeYaw(u) - _computeYaw(u - _miniStep)) / (10. * _miniStep);
    double rightFirstDerivative = std::abs(_computeYaw(u + _miniStep) - _computeYaw(u)) / (10. * _miniStep);
    vpp.firstDerivative = 0.5 * (leftFirstDerivative + rightFirstDerivative);
    vpp.secondDerivative = std::abs(rightFirstDerivative - leftFirstDerivative) / (10. * _miniStep);
    vpp.curvature = std::abs(vpp.secondDerivative) / std::pow(1. + std::pow(vpp.firstDerivative, 2.), 1.5);
    return vpp.curvature;
}

std::valarray<double> Trajectory::computeTimeProfile()
{
    _totalTime = 0;
    _relativeTotalTime = 0;
    _times = std::valarray<double>(Parameters::VelocityPlanningPoints + 1);
    _times[0] = 0;

    for (unsigned int i = 1; i <= Parameters::VelocityPlanningPoints; ++i)
    {
        auto &p = _planningPoints[i];
        auto &lastp = _planningPoints[i - 1];
        double deltaArc = p.u - lastp.u;
        double deltaTime = 0;
        if (p.velocity + lastp.velocity <= 0.)
        {
            ROS_ERROR("Error: p.velocity + lastp.velocity <= 0");
        }
        else
        {
            deltaTime = (2. * deltaArc / (p.velocity + lastp.velocity));
        }
        _totalTime += deltaTime;
        _relativeTotalTime += p.velocity;
        _times[i] = _totalTime; //_relativeTotalTime;
    }

    //_times *= _totalTime / _relativeTotalTime;
    for (unsigned int i = 0; i <= Parameters::VelocityPlanningPoints; ++i)
        _planningPoints[i].time = _times[i];

    return _times;
}

bool Trajectory::saveProfile(const std::string &csvfilename)
{
    std::fstream f;
    f.open(csvfilename, std::ios::out);

    if (f.bad())
        return false;

    for (unsigned int i = 0; i <= Parameters::VelocityPlanningPoints; ++i)
    {
        auto &p = _planningPoints[i];
        f << _computeYaw(p.splineU) << "," << p.firstDerivative << ","
          << p.secondDerivative << "," << p.curvature << ","
          << p.velocity << "," << p.maxVelocity << ","
          << p.point.x << "," << p.point.y
          #if (DIMENSIONS == 3)
          << "," << p.point.z
          #endif
          << std::endl;
    }

    f.close();
    ROS_INFO_STREAM("Saved profile to " << csvfilename);
    return true;
}
