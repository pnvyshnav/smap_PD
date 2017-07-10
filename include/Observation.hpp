#pragma once

#include <vector>
#include <memory>

#include "Parameters.hpp"
#include "SensorRay.hpp"
#include "Observable.hpp"

enum GeometryType
{
    GEOMETRY_VOXEL,
    GEOMETRY_SPURIOUS,
    GEOMETRY_HOLE
};

struct Measurement
{
    /**
     * Measurement value, e.g. measured distance from a range sensor.
     */
    Parameters::NumType value;

    /**
     * Ray of sensor that produced this measurement.
     */
    SensorRay sensor;

    /**
     * Determines which geometry was sensed.
     */
    GeometryType geometry;

    static Measurement hole(const SensorRay &sensor)
    {
        return Measurement(sensor, std::numeric_limits<Parameters::NumType>::infinity(), GEOMETRY_HOLE);
    }

    static Measurement spurious(const SensorRay &sensor)
    {
        return Measurement(sensor, std::numeric_limits<Parameters::NumType>::infinity(), GEOMETRY_SPURIOUS);
    }

    static Measurement voxel(const SensorRay &sensor, Parameters::NumType value)
    {
        return Measurement(sensor, value);
    }

private:
    Measurement(const SensorRay sensor, Parameters::NumType value, GeometryType geometry = GEOMETRY_VOXEL)
            : sensor(sensor), value(value), geometry(geometry)
    {}
};

class Observation : public Observable
{
public:
    Observation(const std::vector<Measurement> &measurements = std::vector<Measurement>())
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

    /**
     * Append observation measurements to these measurements.
     * @param observation The observation measurements to add.
     */
    void append(const Observation &observation)
    {
        _measurements.insert(_measurements.end(),
                             observation._measurements.begin(),
                             observation._measurements.end());
    }

    /**
     * Randomly extracts measurements from the observation up to the
     * given ratio.
     * @param ratio Amount of samples in [0,1] to be selected.
     * @param seed Seed value for the random number generator.
     * @return New observation instance.
     */
    Observation take(float ratio, unsigned int seed = 123) const
    {
        std::vector<int> indices(_measurements.size());
        std::iota(indices.begin(), indices.end(), 0);
        std::srand(seed);
        auto limit = (unsigned int) (ratio * _measurements.size());
        for (unsigned int i = 0; i < limit; ++i)
            std::swap(indices[i], indices[std::rand() % _measurements.size()]);
        std::vector<Measurement> measurements;
        for (unsigned int i = 0; i < limit; ++i)
            measurements.push_back(_measurements[indices[i]]);
        return Observation(measurements);
    }

    octomap::Pointcloud pointcloud() const
    {
        octomap::Pointcloud cloud;
        for (auto & measurement : _measurements)
        {
            auto &s = measurement.sensor;
            auto endpoint = s.position + s.orientation * s.range;
            cloud.push_back(endpoint);
        }
        return cloud;
    }

private:
    std::vector<Measurement> _measurements;
};