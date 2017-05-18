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

Belief::Belief(bool empty) : _recomputeMean(true), _recomputeVariance(true),
                             _useStored(false), _meanLocked(false), _empty(empty)
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


    // only defining mean (linear formulation)
    double a = 6. * (1. - n - 2*Parameters::priorMean + 2*n*Parameters::priorMean)/(n * (1 + n));
    double b = (-2. * (1. - 2. * n - 3*Parameters::priorMean + 3*n*Parameters::priorMean))/(n * (1 + n));
    pdf = b + a * particles;
//
//    pdf = {
//            5.42912966e-02,   5.00027027e-02,   4.59304519e-02,   4.20679897e-02,
//            3.84088281e-02,   3.49465470e-02,   3.16747944e-02,   2.85872874e-02,
//            2.56778132e-02,   2.29402295e-02,   2.03684657e-02,   1.79565236e-02,
//            1.56984784e-02,   1.35884794e-02,   1.16207515e-02,   9.78959570e-03,
//            8.08939052e-03,   6.51459292e-03,   5.05973957e-03,   3.71944801e-03,
//            2.48841795e-03,   1.36143261e-03,   3.33360098e-04,   6.93857374e-06,
//            6.93857374e-06,   6.93857374e-06,   6.93857374e-06,   6.93857374e-06,
//            6.93857374e-06,   6.93857374e-06,   6.93857374e-06,   6.93857374e-06,
//            6.93857374e-06,   6.93857374e-06,   6.93857374e-06,   6.93857374e-06,
//            6.93857374e-06,   6.93857374e-06,   6.93857374e-06,   6.93857374e-06,
//            6.93857374e-06,   6.93857374e-06,   6.93857374e-06,   6.93857374e-06,
//            6.93857374e-06,   6.93857374e-06,   6.93857374e-06,   6.93857374e-06,
//            6.93857374e-06,   6.93857374e-06,   6.93857374e-06,   6.93857374e-06,
//            6.93857374e-06,   6.93857374e-06,   6.93857374e-06,   6.93857374e-06,
//            6.93857374e-06,   6.93857374e-06,   6.93857374e-06,   6.93857374e-06,
//            6.93857374e-06,   6.93857374e-06,   6.93857374e-06,   6.93857374e-06,
//            6.93857374e-06,   6.93857374e-06,   6.93857374e-06,   6.93857374e-06,
//            6.93857374e-06,   6.93857374e-06,   6.93857374e-06,   6.93857374e-06,
//            6.93857374e-06,   6.93857374e-06,   6.73843932e-04,   1.50430027e-03,
//            2.41402169e-03,   3.40778488e-03,   4.49045783e-03,   5.66699812e-03,
//            6.94245124e-03,   8.32194908e-03,   9.81070841e-03,   1.14140295e-02,
//            1.31372949e-02,   1.49859679e-02,   1.69655914e-02,   1.90817871e-02,
//            2.13402539e-02,   2.37467669e-02,   2.63071768e-02,   2.90274084e-02,
//            3.19134598e-02,   3.49714018e-02,   3.82073765e-02,   3.90071632e-02,
//            3.79234302e-02,   3.68396971e-02,   3.57559641e-02,   3.46722311e-02
//    };

    // consider desired mean and variance (quadratic formulation)
//    double mean = Parameters::priorMean;
//    double variance = std::pow(Parameters::priorStd, 2.);
//    double a, b, c;
//    a = -((6 * (6 + n * (21 - 22 * mean) + 30 * std::pow(n, 4.) * std::pow(mean, 2.)
//                + std::pow(n, 2.) * (21 - 60 * mean + 30 * variance)
//                + std::pow(n, 3.) * (6 - 32 * mean + 30 * std::pow(mean, 2.) + 30 * variance)))
//          / (4 - 5 * std::pow(n, 2.) + std::pow(n, 4.)));
//    b = (6 + n * (9 - 18 * mean) + 30 * std::pow(n, 3.) * std::pow(mean, 2.) +
//         std::pow(n, 2.) * (9 - 36 * mean + 30 * variance))
//        / ((-2 + n) * (-1 + n) * n);
//    c = (30 * n * (2 + n * (3 - 6 * mean) + 6 * std::pow(n, 3.) * std::pow(mean, 2.) +
//                   std::pow(n, 2.) * (1 - 6 * mean + 6 * variance)))
//        / (4 - 5 * std::pow(n, 2.) + std::pow(n, 4.));
//
//    pdf = std::valarray<double>(particles.size());
//    for (unsigned int i = 0; i < particles.size(); ++i)
//        pdf[i] = b + a * particles[i] + c * std::pow(particles[i], 2.);

    //double a = (6. * std::pow(n, 2) * Parameters::priorMean)/(1. + 3. * n + 2 * std::pow(n, 2));
    //double a = (6.*n*Parameters::priorMean)/(1.+3.*n+2.*std::pow(n, 2));

    //pdf = a * particles;

    // TODO reactivate
    /*if (!isBeliefValid())
    {
        ROS_ERROR("Prior voxel belief is invalid.");
    }*/
}

bool Belief::empty()
{
    return _empty;
}

Parameters::NumType Belief::mean()
{
    if (_useStored || _meanLocked)
        return _mean;
    if (_recomputeMean)
    {
        _mean = (particles * pdf).sum();
        _recomputeMean = false;
    }
    return _mean;
}

Parameters::NumType Belief::variance()
{
    if (_useStored)
        return _variance;
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

void Belief::storeMeanVariance(double mean, double variance)
{
    _mean = mean;
    _variance = variance;
    _useStored = true;
}

void Belief::lockMean()
{
    _meanLocked = true;
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
    if (ps > 0.)
        pdf = new_pdf / ps;
//    else
//        ROS_WARN("Cannot update belief. New PDF sum (%g) would be too small.", new_pdf.sum());

    //assert(isBeliefValid());
    _recomputeMean = true;
    _recomputeVariance = true;
}

bool Belief::operator==(const Belief &rhs) const
{
    return std::abs(_constMean() - rhs._constMean()) < Parameters::equalityThreshold
           && std::abs(_constVariance() - rhs._constVariance()) < Parameters::equalityThreshold;
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

Parameters::NumType Belief::_constMean() const
{
    if (_useStored || _meanLocked)
        return _mean;
    return (particles * pdf).sum();
}

Parameters::NumType Belief::_constVariance() const
{
    if (_useStored)
        return _variance;
    Parameters::NumType exp = 0;
    for (unsigned int i = 0; i < Parameters::numParticles; ++i)
    {
        exp += std::pow(particles[i], 2.) * pdf[i];
    }
    return exp - std::pow(_constMean(), 2.);
}