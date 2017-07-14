#pragma once

#include <cmath>
#include <Eigen/Dense>
#include <vector>
#include <initializer_list>

#include "Observable.hpp"
#include "TrueMap.h"

struct TrajectoryPoint
{
    Eigen::Vector3f position;
    Eigen::Vector3f orientation;

    // theta is the 2d orientation angle in radians
    TrajectoryPoint(Eigen::Vector3f position, float theta)
            : position(position), orientation(std::cos(theta), std::sin(theta), 0.f)
    {}

    TrajectoryPoint(Eigen::Vector3f position, Eigen::Vector3f orientation)
            : position(position), orientation(orientation)
    {}

    Parameters::Vec3Type robotPosition() const
    {
        return Parameters::Vec3Type(position[0], position[1], position[2]);
    }

    Parameters::Vec3Type robotOrientation() const
    {
        return Parameters::Vec3Type(orientation[0], orientation[1], orientation[2]);
    }
};

class Trajectory : public Observable
{
public:
    typedef std::vector<TrajectoryPoint>::iterator iterator;

    Trajectory(std::vector<TrajectoryPoint> points = std::vector<TrajectoryPoint>())
            : _points(points)
    {}

    Trajectory(std::initializer_list<TrajectoryPoint> points)
            : _points(points)
    {}

    iterator begin()
    {
        return _points.begin();
    }

    iterator end()
    {
        return _points.end();
    }

    void clear()
    {
        _points.clear();
    }

    void add(const TrajectoryPoint &point)
    {
        _points.push_back(point);
    }

    Trajectory& operator=(const Trajectory &rhs)
    {
        _points = rhs._points;
        return *this;
    }

    size_t size() const
    {
        return _points.size();
    }

    Box boundingBox() const
    {
        Box bounds;
        bounds.x1 = bounds.y1 = std::numeric_limits<double>::max();
        bounds.x2 = bounds.y2 = std::numeric_limits<double>::min();
        for (auto &p : _points)
            bounds.update(p.position[0], p.position[1]);
        return bounds;
    }

private:
    std::vector<TrajectoryPoint> _points;

};
