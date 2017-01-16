#pragma once

#include <initializer_list>
#include <valarray>

#include "tinysplinecpp.h"
#include "Parameters.hpp"
#include "BeliefMap.h"

#define DIMENSIONS 2

struct Point
{
    double x;
    double y;

#if (DIMENSIONS == 3)
    double z;
    Point(double x, double y, double z) : x(x), y(y), z(z)
    {}
#endif

    Point()
    {}

    Point(double x, double y) : x(x), y(y)
    {}
};

struct TrajectoryEvaluationResult
{
    Point point;
    double yaw;
    double arcLength;
    double u;
    double time;
    double velocity;
    TrajectoryEvaluationResult(const Point &point, double yaw, double arcLength, double u)
            : point(point), yaw(yaw), arcLength(arcLength), u(u)
    {}
};

struct VelocityPlanningParameters
{
    double startVelocity = 0;
    double endVelocity = 0;
    double acceleration = 6;
    double maxVelocity = 1.5;
    VelocityPlanningParameters()
    {}
};

class Trajectory
{
    friend class TrajectoryPlanner;
public:
    TrajectoryEvaluationResult evaluate(double u, bool computeTime = false);
    double evaluateCurvature(double u);

    struct VelocityPlanningPoint
    {
        double u;

        double velocity;
        double maxVelocity;
        double curvature;

        double firstDerivative;
        double secondDerivative;

        Point point;
    };

    /**
     * Computes velocity planning points, stores them internally and returns them.
     * @param parameters Configuration for velocity planning.
     * @return Velocity planning points.
     */
    std::vector<VelocityPlanningPoint> computeVelocities(
            const VelocityPlanningParameters &parameters = VelocityPlanningParameters());

    /**
     * Computes timing for the internally stored velocity planning points.
     * @return Time stamp (starting at 0) for each planning point.
     */
    std::valarray<double> computeTimeProfile();

    /**
     * Returns keys of the voxels which are visited by the trajectory.
     * @param trueMap OctoMap as reference for voxel key generation.
     * @return Set of OctoMap voxel keys.
     */
    Parameters::KeySet computeSplineVoxels(TrueMap &trueMap);

    /**
     * Returns the control points of this spline.
     * @return Spline control points.
     */
    const std::vector<Point> controlPoints();

    /**
     * Returns the keys to the voxels covered by the current spline.
     * @return Set of distinct OcTreeKeys.
     */
    const Parameters::KeySet &splineVoxels() const
    {
        return _splineVoxels;
    }

    /**
     * Returns the keys to the voxels covered within the next Parameters::EvaluateFutureSteps by the current spline.
     * @return Set of distinct OcTreeKeys.
     */
    const Parameters::KeySet &splineFutureVoxels() const
    {
        return _splineFutureVoxels;
    }

    const double totalArcLength() const
    {
        return _totalArcLength;
    }

    const double totalTime() const
    {
        return _totalTime;
    }

    const double relativeTotalTime() const
    {
        return _relativeTotalTime;
    }

private:
    Trajectory(std::initializer_list<Point> points, unsigned int degree = 2);
    ts::BSpline _spline;

    double _computeYaw(double u);
    bool _yawWasPi;

    double _lastU;

    double _totalArcLength;
    double _totalTime;
    double _relativeTotalTime;

    double _maxRad;
    double _smallStep;
    double _miniStep;

    Parameters::KeySet _splineVoxels;
    Parameters::KeySet _splineFutureVoxels;

    std::vector<VelocityPlanningPoint> _planningPoints;
    std::valarray<double> _times;
};