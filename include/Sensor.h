#pragma once

#include "Parameters.hpp"
#include "Observation.hpp"
#include "TrueMap.h"
#include "BeliefMap.h"

/**
 * Class representing the results of computing the inverse sensor cause model.
 */
struct InverseCauseModel
{
    InverseCauseModel()
    {}

    InverseCauseModel(const InverseCauseModel &icm)
            : posteriorOnRay(icm.posteriorOnRay),
              posteriorInfinity(icm.posteriorInfinity),
              rayLength(icm.rayLength)
    {
        for (auto &key : icm.ray)
            ray.push_back(octomap::OcTreeKey(key));
    }

    std::valarray<Parameters::NumType> posteriorOnRay;
    Parameters::NumType posteriorInfinity;
    std::vector<octomap::OcTreeKey> ray;
    unsigned int rayLength;
};

class Sensor : public Visualizable
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
    virtual Parameters::NumType likelihoodGivenCause(Measurement measurement, QVoxel causeVoxel) const;

    virtual InverseCauseModel *computeInverseCauseModel(Measurement measurement, BeliefMap &beliefMap) const;

protected:
    Parameters::Vec3Type _position;
    Parameters::Vec3Type _orientation;
    const Parameters::NumType _range;
};
