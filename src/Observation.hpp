#pragma once

#include <vector>

#include "Parameters.hpp"

class Observation {
public:
    Observation(const std::vector<Parameters::NumType> &measurements);

    const std::vector<Parameters::NumType> &measurements() const;

private:
    std::vector<Parameters::NumType> _measurements;
};
