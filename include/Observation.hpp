#pragma once

#include <vector>

#include "Parameters.hpp"
#include "TrueMap.h"

struct Measurement
{
    /**
     * Measurement value, e.g. measured distance from a range sensor.
     */
    const Parameters::NumType value;

    /**
     * Determines which geometry was sensed.
     */
    const GeometryType geometry;

    static Measurement hole()
    {
        return Measurement(std::numeric_limits<Parameters::NumType>::infinity(), GEOMETRY_HOLE);
    }

    static Measurement spurious()
    {
        return Measurement(std::numeric_limits<Parameters::NumType>::infinity(), GEOMETRY_SPURIOUS);
    }

    static Measurement voxel(Parameters::NumType value)
    {
        return Measurement(value);
    }

private:
    Measurement(Parameters::NumType value, GeometryType geometry = GEOMETRY_VOXEL)
            : value(value), geometry(geometry)
    {}
};

class Observation
{
public:
    Observation(const std::vector<Measurement> &measurements)
            : _measurements(measurements)
    {}

    const std::vector<Measurement> &measurements() const
    {
        return _measurements;
    }

private:
    std::vector<Measurement> _measurements;
};
