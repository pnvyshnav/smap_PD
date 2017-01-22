#include <ros/console.h>
#include <sstream>

#include "../include/BeliefVoxel.h"
#include "../include/Parameters.hpp"


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

    double n = Parameters::numParticles;
    //TODO bugfix: take (1-prior) such that mean matches priorMean
    //Parameters::NumType c1 = (Parameters::NumType) ((4. * n - 2. - 6. * (n - 1) * (1.-Parameters::priorMean)) / (n * n + n));
    //Parameters::NumType c2 = (Parameters::NumType) (-c1 + 2. / n);
    //double a = c2 - (c2-c1);
    //pdf = c2 - (c2-c1) * particles;

//    double a = (6. * (-1 - n + 2.*n*Parameters::priorMean))/(-1. + std::pow(n, 2.));
//    double b = (-2. * (-1 - 2.*n + 3*n*Parameters::priorMean))/((-1 + n) * n);

   // double a = 6. * (1. - n + 2. * n * Parameters::priorMean)/(-1. + std::pow(n, 2));
   // double b = -2. * (1. - 2. * n + 3. * n * Parameters::priorMean)/(n * (1. + n));
    double a = 6. * (1. - n - 2*Parameters::priorMean + 2*n*Parameters::priorMean)/(n * (1 + n));
    double b = (-2. * (1. - 2. * n - 3*Parameters::priorMean + 3*n*Parameters::priorMean))/(n * (1 + n));
    pdf = b + a * particles;

    //double a = (6. * std::pow(n, 2) * Parameters::priorMean)/(1. + 3. * n + 2 * std::pow(n, 2));
    //double a = (6.*n*Parameters::priorMean)/(1.+3.*n+2.*std::pow(n, 2));

    //pdf = a * particles;

    // TODO reactivate
//    if (!isBeliefValid())
//    {
//        ROS_ERROR("Prior voxel belief is invalid.");
//    }
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
            exp += std::pow(particles[i], 2.) * pdf[i];
        }
        _variance = exp - std::pow(mean(), 2.);
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
    return std::abs(pdf.sum() - 1.) < 1e-10;
}

void Belief::updateBelief(Parameters::NumType a, Parameters::NumType b)
{
    const std::valarray<Parameters::NumType> new_pdf = (a * particles + b) * pdf;

    // normalize
    const double ps = new_pdf.sum();
    if (ps > 0.) // TODO remove condition
        pdf = new_pdf / ps;
    else
        ROS_WARN("Cannot update belief. New PDF sum (%g) would be too small.", new_pdf.sum());

    //assert(isBeliefValid()); // TODO reactivate
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

void Belief::reset()
{
    double n = Parameters::numParticles;
    double a = 6. * (1. - n - 2*Parameters::priorMean + 2*n*Parameters::priorMean)/(n * (1 + n));
    double b = (-2. * (1. - 2. * n - 3*Parameters::priorMean + 3*n*Parameters::priorMean))/(n * (1 + n));
    pdf = b + a * particles;
    _recomputeMean = true;
    _recomputeVariance = true;
}