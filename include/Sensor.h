#pragma once

#include "Parameters.hpp"
#include "Observation.hpp"
#include "TrueMap.h"
#include "BeliefMap.h"

/**
 * Abstract class representing a sensor.
 */
class Sensor
{
public:
    Sensor(Parameters::Vec3Type &position,
           Parameters::Vec3Type &orientation);

    virtual Observation observe(TrueMap &trueMap) const = 0;

    /**
     * @brief Global position of the sensor.
     */
    Parameters::Vec3Type position() const;
    void setPosition(const Parameters::Vec3Type &position);

    /**
     * @brief Normalized vector describing the orientation of the sensor.
     */
    Parameters::Vec3Type orientation() const;
    void setOrientation(const Parameters::Vec3Type &orientation);

    /**
     * @brief Computes the likelihood of the measurement given the true voxel that caused the measurement.
     * @param measurement The measurement.
     * @param causeVoxel The true cause voxel.
     * @return Likelihood between 0 and 1.
     */
    virtual Parameters::NumType likelihoodGivenCause(Measurement measurement, QTrueVoxel causeVoxel) const = 0;

    /**
     * Class representing the results of computing the inverse cause model.
     */
    struct InverseCauseModel
    {
        std::valarray<Parameters::NumType> posteriorOnRay;
        Parameters::NumType posteriorInfinity;
        std::vector<octomap::OcTreeKey> voxelKeys;
    };

    InverseCauseModel computeInverseCauseModel(Measurement measurement, TrueMap &trueMap, BeliefMap &beliefMap) const;

protected:
    Parameters::Vec3Type _position;
    Parameters::Vec3Type _orientation;
};
