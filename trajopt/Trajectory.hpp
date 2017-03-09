#pragma once

#include <Eigen/Core>

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

    Point() : x(0), y(0)
#if (DIMENSIONS == 3)
    , z(0)
#endif
    {}

    Point(double x, double y) : x(x), y(y)
    {}

    /**
     * Computes the Geometric Mean.
     * @return L2 norm.
     */
    double norm() const
    {
#if (DIMENSIONS == 3)
        return std::sqrt(std::pow(x, 2.) + std::pow(y, 2.) + std::pow(z, 2.));
#else
        return std::sqrt(std::pow(x, 2.) + std::pow(y, 2.));
#endif
    }

    double dist(const Point &p) const
    {

#if (DIMENSIONS == 3)
        return std::sqrt(std::pow(x-p.x, 2.) + std::pow(y-p.y, 2.) + std::pow(z-p.z, 2.));
#else
        return std::sqrt(std::pow(x-p.x, 2.) + std::pow(y-p.y, 2.));
#endif
    }
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
    double acceleration;

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
protected:
    Trajectory(bool empty) : _empty(empty)
    {}

public:
    virtual TrajectoryEvaluationResult evaluate(double u, bool computeTime = false) = 0;
    virtual TrajectoryEvaluationResult evaluateAtTime(double time) = 0;

    /**
     * Number of definable parameters.
     * @return Degrees of freedom.
     */
    virtual long dof() const
    {
        return 0;
    }

    /**
     * Parameterize the trajectory by a vector of length given by DOF.
     * @param parameter The parameter row vector.
     */
    virtual void parameterize(const Eigen::VectorXd &parameter)
    {}

    virtual Point start() const = 0;
    virtual Point end() const = 0;

    virtual double totalTime() const = 0;

    const bool empty() const
    {
        return _empty;
    }

private:
    bool _empty;
};

