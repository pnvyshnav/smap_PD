#include "../include/BeliefVoxel.h"

#include <valarray>
#include <algorithm>
#include <cmath>
#include <cassert>

#include <octomap/OcTreeDataNode.h>

#include "../include/Parameters.hpp"

const Belief::Particles Belief::particles = Belief::generateParticles();


Belief::Belief()
{
    unsigned int n = Parameters::numParticles;
    Parameters::NumType c1 = (Parameters::NumType) ((4. * n - 2. - 6. * (n - 1) * Parameters::priorMean) / (n * n + n));
    Parameters::NumType c2 = (Parameters::NumType) (-c1 + 2. / n);
    pdf = c2-(c2-c1)*particles;
}

Parameters::NumType Belief::mean() const
{
    return (particles * pdf).sum();
}

Parameters::NumType Belief::variance() const
{
    Parameters::NumType exp = 0;
    for (unsigned int i = 0; i < Parameters::numParticles; ++i)
    {
        exp += std::pow(particles[i], 2) * pdf[i];
    }
    return exp - std::pow(mean(), 2);
}

bool Belief::isBeliefValid() const
{
    for (Parameters::NumType p : pdf)
    {
        if (p < 0)
            return false;
    }
    return pdf.sum() < 1e-10;
}

void Belief::updateBelief(Parameters::NumType a, Parameters::NumType b)
{
    pdf = (a * particles + b) * pdf;

    // normalize
    pdf /= pdf.sum();

    //TODO self.needRefresh = 1 ?

    assert(isBeliefValid());
}

bool Belief::operator==(const Belief &rhs) const
{
    return std::abs(mean() - rhs.mean()) < Parameters::equalityThreshold
        && std::abs(variance() - rhs.variance()) < Parameters::equalityThreshold;
}

Belief::Particles Belief::generateParticles()
{
    Belief::Particles particles(Parameters::numParticles);

    // initialize particles ranging uniformly from 0 to 1 (including 1)
    Parameters::NumType delta = (Parameters::NumType) (1. / (Parameters::numParticles - 1));
    Parameters::NumType x = 0;
    for (auto &particle : particles)
        particle = x += delta;

    return particles;
}
