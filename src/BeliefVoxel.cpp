#include <ros/console.h>
#include <sstream>

#include "../include/BeliefVoxel.h"


Belief::Particles generateParticles()
{
    Belief::Particles particles(Parameters::numParticles);

    // initialize particles ranging uniformly from 0 to 1 (including 1)
    Parameters::NumType delta = (Parameters::NumType) (1. / (Parameters::numParticles - 1));
    for (unsigned int i = 0; i < Parameters::numParticles; ++i)
        particles[i] = i * delta;

    return particles;
}

Belief::Particles particles = generateParticles();


Belief::Belief() : _recomputeMean(true), _recomputeVariance(true)
{
    if (particles.size() == 0)
        particles = generateParticles();

    unsigned int n = Parameters::numParticles;
    Parameters::NumType c1 = (Parameters::NumType) ((4. * n - 2. - 6. * (n - 1) * Parameters::priorMean) / (n * n + n));
    Parameters::NumType c2 = (Parameters::NumType) (-c1 + 2. / n);
    pdf = c2 - (c2-c1) * particles;
    if (!isBeliefValid())
    {
        ROS_ERROR("Prior voxel belief is invalid.");
    }
}

Parameters::NumType Belief::mean()
{
    if (_recomputeMean)
    {
        _mean = (particles * pdf).sum();
        _recomputeMean = false;
    }
    return _mean;
}

Parameters::NumType Belief::variance()
{
    if (_recomputeVariance)
    {
        Parameters::NumType exp = 0;
        for (unsigned int i = 0; i < Parameters::numParticles; ++i)
        {
            exp += std::pow(particles[i], 2) * pdf[i];
        }
        _variance = exp - std::pow(mean(), 2);
        _recomputeVariance = false;
    }
    return _variance;
}

bool Belief::isBeliefValid() const
{
    for (Parameters::NumType p : pdf)
    {
        if (p < -1e-10)
            return false;
    }
    return pdf.sum() - 1. < 1e-10;
}

void Belief::updateBelief(Parameters::NumType a, Parameters::NumType b)
{
    const std::valarray<Parameters::NumType> new_pdf = (a * particles + b) * pdf;

    // normalize
    if (new_pdf.sum() > 0.) // TODO remove condition
        pdf = new_pdf / new_pdf.sum();
    else
        ROS_WARN("Cannot update belief. New PDF sum (%g) would be too small.", new_pdf.sum());

    assert(isBeliefValid());
    _recomputeMean = true;
    _recomputeVariance = true;
}

bool Belief::operator==(Belief &rhs)
{
    return std::abs(mean() - rhs.mean()) < Parameters::equalityThreshold
        && std::abs(variance() - rhs.variance()) < Parameters::equalityThreshold;
}

const std::string Belief::str() const
{
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2);
    for (auto &p : pdf)
        ss << p << " ";
    return ss.str();
}
