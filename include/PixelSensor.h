#pragma once

#include "Parameters.hpp"
#include "TrueMap.h"


/**
 * @brief Represents a range sensor that only computes a single ray.
 */
class PixelSensor {
public:
    PixelSensor(Parameters::Vec3Type direction, Parameters::Vec3Type position);

    /**
     * @brief Normalized vector describing the orientation of the pixel sensor.
     */
    Parameters::Vec3Type direction() const;
    void setDirection(Parameters::Vec3Type direction);


    /**
     * @brief Global position of the pixel sensor.
     */
    Parameters::Vec3Type position() const;
    void setPosition(Parameters::Vec3Type position);



    /**
     * @brief Simulates a range sensor measurement given the true map. 
     * 
     * @param trueMap The true occupancy map.
     * @return Scalar representing the simulated distance measurent.
     */
    Parameters::NumType observe(TrueMap &trueMap) const;

private:
    Parameters::Vec3Type _direction;
    Parameters::Vec3Type _position;

    Parameters::NumType _observeGivenCause(TrueVoxel &causeVoxel) const;
};
