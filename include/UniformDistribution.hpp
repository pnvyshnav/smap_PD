#pragma once

#include <random>

#include "Parameters.hpp"

/**
 * Uniform distribution from 0 to 1.
 */
class UniformDistribution
{
public:
    static Parameters::NumType sample()
    {
        return _uniform(_generator);
    }

private:
    __attribute__((weak))
    static std::uniform_real_distribution<Parameters::NumType> _uniform;
    __attribute__((weak))
    static std::mt19937 _generator;
    __attribute__((weak))
    static std::random_device _rd;
};

std::uniform_real_distribution<Parameters::NumType> UniformDistribution::_uniform = std::uniform_real_distribution<Parameters::NumType>(0, 1);
std::random_device UniformDistribution::_rd;
std::mt19937 UniformDistribution::_generator = std::mt19937(UniformDistribution::_rd());
