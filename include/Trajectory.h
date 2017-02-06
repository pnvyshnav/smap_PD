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
    double splineU;
    double time;
    double velocity;

    bool empty;

    TrajectoryEvaluationResult() : empty(true)
    {}

    TrajectoryEvaluationResult(const Point &point, double yaw, double arcLength,
                               double u, double splineU, double velocity)
            : point(point), yaw(yaw), arcLength(arcLength),
              u(u), splineU(splineU), empty(false), velocity(velocity)
    {}
};

struct VelocityPlanningParameters
{
    double startVelocity = 0;
    double endVelocity = 0;
    double acceleration = 5;
    double maxVelocity = 1.1;
    double minVelocity = 0.05;
    double maxRotationalVelocity = 0.2;

    VelocityPlanningParameters()
    {}
};

/**
 * Trajectory planning using cubic splines.
 */
class Trajectory
{
    friend class TrajectoryPlanner;
public:
    Trajectory();
    Trajectory(const Trajectory &trajectory);

    TrajectoryEvaluationResult evaluate(double u, bool computeTime = false);

    TrajectoryEvaluationResult evaluateAtTime(double time);

    struct VelocityPlanningPoint
    {
        double u;
        double splineU;
        double time;

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

    bool saveProfile(const std::string &csvfilename);

    /**
     * Returns keys of the voxels which are visited by the trajectory.
     * @param trueMap OctoMap as reference for voxel key generation.
     * @return Set of OctoMap voxel keys.
     */
    Parameters::KeySet splineVoxelKeys(const TrueMap &trueMap);

    /**
     * Returns positions of the voxels which are visited by the trajectory.
     * @param trueMap OctoMap as reference for voxel key generation.
     * @return Set of OctoMap 3d vertices.
     */
    Parameters::PositionSet splineVoxelPositions(const TrueMap &trueMap);

    /**
     * Returns the control points of this spline.
     * @return Spline control points.
     */
    const std::vector<Point> controlPoints() const
    {
        return _controlPoints;
    }

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

    const bool empty() const
    {
        return _empty;
    }

    bool isValid() const;

    const Point start() const
    {
        return _start;
    }

    const Point end() const
    {
        return _end;
    }

private:
    Trajectory(std::initializer_list<Point> points, unsigned int degree = 2);
    ts::BSpline _spline;
    std::vector<Point> _controlPoints;

    bool _empty;

    double _computeCurvature(double u, VelocityPlanningPoint &vpp);

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
    Parameters::PositionSet _splineVoxelPositions;
    Parameters::KeySet _splineFutureVoxels;

    std::vector<VelocityPlanningPoint> _planningPoints;
    std::valarray<double> _times;

    Point _start;
    Point _end;
};