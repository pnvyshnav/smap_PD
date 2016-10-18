#pragma once

#include <vector>

#include "Parameters.hpp"

class Observation {
public:
    Observation(const std::vector<Parameters::NumType> &measurements)
    : _measurements(measurements) {}

    const std::vector<Parameters::NumType> &measurements() const
    {
        return _measurements;
    }

private:
    std::vector<Parameters::NumType> _measurements;
};
