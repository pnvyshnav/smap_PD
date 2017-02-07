#pragma once


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

class Trajectory
{
    friend class TrajectoryPlanner;
public:
    virtual TrajectoryEvaluationResult evaluate(double u, bool computeTime = false) = 0;
    virtual TrajectoryEvaluationResult evaluateAtTime(double time) = 0;
};

