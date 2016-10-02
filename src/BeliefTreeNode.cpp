#include "BeliefTreeNode.h"

#include <array>
#include <algorithm>
#include <cmath>

#include <octomap/OcTreeDataNode.h>

#include "parameters.hpp"

const Belief::Particles Belief::particles = Belief::generateParticles();


Belief::Belief()
{
    ///TODO initialize pdf
}

Parameters::NumType Belief::mean() const
{
    Parameters::NumType sum = 0.;
    auto ipar = particles.cbegin();
    auto ipdf = pdf.cbegin();
    while (ipar != particles.cend() && ipdf != pdf.cend())
        sum += *ipar * *ipdf;
    return sum;
}

Parameters::NumType Belief::variance() const
{
    Parameters::NumType exp = 0.;
    auto ipar = particles.cbegin();
    auto ipdf = pdf.cbegin();
    while (ipar != particles.cend() && ipdf != pdf.cend())
        exp += std::pow(*ipar, 2.) * *ipdf;
    return exp - std::pow(mean(), 2.);
}

bool Belief::isBeliefValid() const
{
    return std::all_of(pdf.cbegin(), pdf.cend(), [](Parameters::NumType p){ return p >= 0; })
        && std::abs(std::accumulate(pdf.cbegin(), pdf.cend(), 0.) - 1.) < 1e-10;
}

Belief::Particles Belief::generateParticles()
{
    Belief::Particles particles;

    // initialize particles ranging uniformly from 0 to 1 (including 1)
    Parameters::NumType delta = 1. / (Parameters::numParticles - 1);
    Parameters::NumType x = 0.;
    for (auto &particle : particles)
        particle = x += delta;

    return particles;
}
