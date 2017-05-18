#pragma once

#include "Parameters.hpp"
#include "Observation.hpp"
#include "TrueMap.h"
#include "BeliefMap.h"

class Sensor : public Observable
{
public:
    Sensor(Parameters::Vec3Type position,
           Parameters::Vec3Type orientation,
           Parameters::NumType range = Parameters::sensorRange);

    Sensor(const Sensor &sensor);

    /**
     * @brief Global position of the sensor.
     */
    Parameters::Vec3Type position() const;
    virtual void setPosition(const Parameters::Vec3Type &position);

    /**
     * @brief Normalized vector describing the orientation of the sensor.
     */
    Parameters::Vec3Type orientation() const;
    virtual void setOrientation(const Parameters::Vec3Type &orientation);


    Parameters::NumType range() const;

    /**
     * @brief Computes the likelihood of the measurement given the actual voxel that caused the measurement.
     * @param measurement The measurement.
     * @param causeVoxel The actual cause voxel.
     * @return Likelihood between 0 and 1.
     */
    static Parameters::NumType likelihoodGivenCause(const Measurement &measurement, const QVoxel &causeVoxel);

    static InverseCauseModel inverseCauseModel(const Measurement &measurement, const BeliefMap &beliefMap);

    inline SensorRay ray() const
    {
        return SensorRay(_position, _orientation, _range);
    }

protected:
    Parameters::Vec3Type _position;
    Parameters::Vec3Type _orientation;
    const Parameters::NumType _range;

private:
    static octomap::KeyRay _keyRay; // avoid wasteful constructions
};
