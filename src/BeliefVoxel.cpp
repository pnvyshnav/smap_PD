#include "../include/BeliefVoxel.h"

#include <array>
#include <algorithm>
#include <cmath>

#include <octomap/OcTreeDataNode.h>

#include "../include/Parameters.hpp"

const Belief::Particles Belief::particles = Belief::generateParticles();


Belief::Belief()
{
    ///TODO initialize pdf
}

Parameters::NumType Belief::mean() const
{
    Parameters::NumType sum = 0.;
    for (auto ipar = particles.cbegin(), ipdf = pdf.cbegin();
         ipar != particles.cend() && ipdf != pdf.cend();
         ++ipar, ++ ipdf)
    {
        sum += *ipar * *ipdf;
    }
    return sum;
}

Parameters::NumType Belief::variance() const
{
    Parameters::NumType exp = 0.;
    for (auto ipar = particles.cbegin(), ipdf = pdf.cbegin();
         ipar != particles.cend() && ipdf != pdf.cend();
         ++ipar, ++ ipdf)
    {
        exp += std::pow(*ipar, 2.) * *ipdf;
    }
    return exp - std::pow(mean(), 2.);
}

bool Belief::isBeliefValid() const
{
    return std::all_of(pdf.cbegin(), pdf.cend(), [](Parameters::NumType p){ return p >= 0; })
        && std::abs(std::accumulate(pdf.cbegin(), pdf.cend(), 0.) - 1.) < 1e-10;
}

void Belief::updateBelief(Parameters::NumType a, Parameters::NumType b)
{
    Parameters::NumType sum = 0.;
    auto ipar = particles.cbegin();
    for (auto ipdf = pdf.begin();
         ipar != particles.cend() && ipdf != pdf.end();
         ++ipar, ++ ipdf)
    {
        auto affineLikelihoodFunction = a * *ipar + b;
        auto unNormalizedPosterior = affineLikelihoodFunction * *ipdf;
        sum += unNormalizedPosterior;
        *ipdf = unNormalizedPosterior;
    }

    // normalize
    for (auto ipdf = pdf.begin(); ipdf != pdf.end(); ++ipdf)
        *ipdf = *ipdf / sum;

    ///TODO self.needRefresh = 1 ?

    assert(isBeliefValid());
}

bool Belief::operator==(const Belief &rhs) const
{
    return std::abs(mean() - rhs.mean()) < Parameters::equalityThreshold
        && std::abs(variance() - rhs.variance()) < Parameters::equalityThreshold;
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
