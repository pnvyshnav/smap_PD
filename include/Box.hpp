#pragma once

#include <sstream>
#include "Parameters.h"


struct Box
{
    double x1, x2;
    double y1, y2;

#if DIMENSIONS == 3
    double z1, z2;
#endif

#if DIMENSIONS == 3
    Box(double x1 = 0, double y1 = 0, double z1 = 0,
        double x2 = 0, double y2 = 0, double z2 = 0)
    : x1(x1), x2(x2), y1(y1), y2(y2), z1(z1), z2(z2)
    {}
#elif DIMENSIONS == 2
    Box(double x1 = 0, double y1 = 0, double x2 = 0, double y2 = 0)
            : x1(x1), x2(x2), y1(y1), y2(y2)
    {}
#endif

    static Box fromXYWH(double x, double y, double width, double height)
    {
        return Box(x, y, x + width, y + height);
    }

#if DIMENSIONS == 3
    static Box fromXYZWHD(double x, double y, double z,
                          double width, double height, double depth)
    {
        return Box(x, y, z, x + width, y + height, z + depth);
    }
#endif

    bool contains(double x, double y) const
    {
        return x >= x1 && x <= x2 && y >= y1 && y <= y2;
    }

#if DIMENSIONS == 3
    bool contains(double x, double y, double z) const
    {
        return x >= x1 && x <= x2 && y >= y1 && y <= y2 && z >= z1 && z <= z2;
    }
#endif

    double width() const
    {
        return x2 - x1;
    }

    double height() const
    {
        return y2 - y1;
    }

#if DIMENSIONS == 3
    double depth() const
    {
        return z2 - z1;
    }
#endif

    void update(double x, double y)
    {
        if (x < x1)
            x1 = x;
        if (y < y1)
            y1 = y;
        if (x > x2)
            x2 = x;
        if (y > y2)
            y2 = y;
    }

#if DIMENSIONS == 3
    void update(double x, double y, double z)
    {
        if (x < x1)
            x1 = x;
        if (y < y1)
            y1 = y;
        if (z < z1)
            z1 = z;
        if (x > x2)
            x2 = x;
        if (y > y2)
            y2 = y;
        if (z > z2)
            z2 = z;
    }
#endif

    std::string str() const
    {
        std::stringstream ss;
        ss << "(" << x1 << ", " << y1;
#if DIMENSIONS == 3
        ss << ", " << z1;
#endif
        ss << ") -- (" << x2 << ", " << y2;
#if DIMENSIONS == 3
        ss << ", " << z2;
#endif
        ss << ")";
        return ss.str();
    }
};
