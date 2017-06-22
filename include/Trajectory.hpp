#pragma once

#include <cmath>
#include <Eigen/Dense>
#include <vector>
#include <initializer_list>

#include "Observable.hpp"

struct TrajectoryPoint
{
    Eigen::Vector3f position;
    Eigen::Vector3f orientation;

    TrajectoryPoint(Eigen::Vector3f position, float zAngleDegrees)
            : position(position)
    {
        auto rotHorizontal = Eigen::AngleAxis<float>(zAngleDegrees * M_PI / 360.f, Eigen::Vector3f(0, 0, 1));
        orientation = rotHorizontal * Eigen::Vector3f(1, 0, 0);
    }

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
    typedef std::vector<TrajectoryPoint>::const_iterator const_iterator;

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

    const_iterator cbegin()
    {
        return _points.cbegin();
    }

    iterator end()
    {
        return _points.end();
    }

    const_iterator cend()
    {
        return _points.cend();
    }

private:
    std::vector<TrajectoryPoint> _points;

};
