#pragma once

#include "parameters.hpp"


class PixelSensor {
public:
    PixelSensor(Parameters::NumType angle);

    Parameters::NumType angle() const;
    void setAngle(Parameters::NumType _angle);

private:
    Parameters::NumType _angle;
};
