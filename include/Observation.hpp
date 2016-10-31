#pragma once

#include <vector>
#include <memory>

#include "Parameters.hpp"
#include "TrueMap.h"

class Sensor;
struct Measurement
{
    /**
     * Measurement value, e.g. measured distance from a range sensor.
     */
    const Parameters::NumType value;

    /**
     * Sensor that produced this measurement.
     */
    const std::shared_ptr<Sensor> sensor;

    /**
     * Determines which geometry was sensed.
     */
    const GeometryType geometry;

    static Measurement hole(const std::shared_ptr<Sensor> &sensor)
    {
        return Measurement(sensor, std::numeric_limits<Parameters::NumType>::infinity(), GEOMETRY_HOLE);
    }

    static Measurement spurious(const std::shared_ptr<Sensor> &sensor)
    {
        return Measurement(sensor, std::numeric_limits<Parameters::NumType>::infinity(), GEOMETRY_SPURIOUS);
    }

    static Measurement voxel(const std::shared_ptr<Sensor> &sensor, Parameters::NumType value)
    {
        return Measurement(sensor, value);
    }

private:
    Measurement(const std::shared_ptr<Sensor> sensor, Parameters::NumType value, GeometryType geometry = GEOMETRY_VOXEL)
            : sensor(sensor), value(value), geometry(geometry)
    {}
};

class Observation
{
public:
    Observation(const std::vector<Measurement> &measurements)
            : _measurements(measurements)
    {}

    /**
     * Allows observation functions to return a single measurement
     * thanks to the implicit constructor.
     */
    Observation(const Measurement &measurement)
    {
        _measurements.push_back(measurement);
    }

    const std::vector<Measurement> &measurements() const
    {
        return _measurements;
    }

private:
    std::vector<Measurement> _measurements;
};
