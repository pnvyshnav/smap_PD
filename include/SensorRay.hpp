#pragma once

#include "Parameters.hpp"

struct SensorRay
{
    Parameters::Vec3Type position;
    Parameters::Vec3Type orientation;
    Parameters::NumType range;

    SensorRay(Parameters::Vec3Type position, Parameters::Vec3Type orientation, Parameters::NumType range)
            : position(position), orientation(orientation), range(range)
    {}
};