#pragma once

#include "Parameters.hpp"

struct SensorRayPoint
{
    Parameters::Vec3Type position;
    double point[3];
    bool occupied;

    SensorRayPoint(const double *point, bool occupied)
            : position((float) point[0], (float) point[1], (float) point[2]),
              occupied(occupied)
    {
        this->point[0] = position.x();
        this->point[1] = position.y();
        this->point[2] = position.z();
    }
};

struct SensorRay
{
    Parameters::Vec3Type position;
    Parameters::Vec3Type orientation;
    Parameters::NumType range;

    SensorRay(Parameters::Vec3Type position, Parameters::Vec3Type orientation, Parameters::NumType range)
            : position(position), orientation(orientation), range(range)
    {}

    std::vector<SensorRayPoint> discretized(double measuredValue) const
    {
        std::vector<SensorRayPoint> points;
        if (measuredValue > range)
            measuredValue = range;
        // compute points on the ray using Bresenham
        double point[3];
        point[0] = position.x();
        point[1] = position.y();
        point[2] = position.z();
        double dx = orientation.x() * measuredValue;
        double dy = orientation.y() * measuredValue;
        double dz = orientation.z() * measuredValue;
        double x_inc = (dx < 0) ? -Parameters::voxelSize : Parameters::voxelSize;
        int l = (int) std::abs(dx / Parameters::voxelSize);
        double y_inc = (dy < 0) ? -Parameters::voxelSize : Parameters::voxelSize;
        int m = (int) std::abs(dy / Parameters::voxelSize);
        double z_inc = (dz < 0) ? -Parameters::voxelSize : Parameters::voxelSize;
        int n = (int) std::abs(dz / Parameters::voxelSize);
        double dx2 = l * 2.;
        double dy2 = m * 2.;
        double dz2 = n * 2.;

        unsigned int i;
        double err_1, err_2;

        if ((l >= m) && (l >= n))
        {
            err_1 = dy2 - l;
            err_2 = dz2 - l;
            for (i = 0; i < l; i++)
            {
                points.push_back(SensorRayPoint(point, false));
                if (err_1 > 0)
                {
                    point[1] += y_inc;
                    err_1 -= dx2;
                }
                if (err_2 > 0)
                {
                    point[2] += z_inc;
                    err_2 -= dx2;
                }
                err_1 += dy2;
                err_2 += dz2;
                point[0] += x_inc;
            }
        }
        else if ((m >= l) && (m >= n))
        {
            err_1 = dx2 - m;
            err_2 = dz2 - m;
            for (i = 0; i < m; i++)
            {
                points.push_back(SensorRayPoint(point, false));
                if (err_1 > 0)
                {
                    point[0] += x_inc;
                    err_1 -= dy2;
                }
                if (err_2 > 0)
                {
                    point[2] += z_inc;
                    err_2 -= dy2;
                }
                err_1 += dx2;
                err_2 += dz2;
                point[1] += y_inc;
            }
        }
        else
        {
            err_1 = dy2 - n;
            err_2 = dx2 - n;
            for (i = 0; i < n; i++)
            {
                points.push_back(SensorRayPoint(point, false));
                if (err_1 > 0)
                {
                    point[1] += y_inc;
                    err_1 -= dz2;
                }
                if (err_2 > 0)
                {
                    point[0] += x_inc;
                    err_2 -= dz2;
                }
                err_1 += dy2;
                err_2 += dx2;
                point[2] += z_inc;
            }
        }
        if (measuredValue < range)
        {
            // end point is occupied
            points.push_back(SensorRayPoint(point, true));
        }
        return points;
    }
};