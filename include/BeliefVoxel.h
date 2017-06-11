#pragma once

#include <valarray>
#include <algorithm>
#include <memory>
#include <cmath>

#include <octomap/OcTreeDataNode.h>

#include "Parameters.hpp"

class Belief
{
public:
    Belief() : _mean(Parameters::priorMean), _variance(std::pow(Parameters::priorStd, 2.))
    {}
    Belief(Parameters::NumType mean, Parameters::NumType variance) :
            _mean(mean), _variance(variance)
    {}

    Parameters::NumType mean()
    {
        return _mean;
    }

    Parameters::NumType variance()
    {
        return _variance;
    }

protected:
    Parameters::NumType _mean, _variance;
};

class BeliefDistribution : public Belief
{
public:
    typedef std::valarray<Parameters::NumType> Particles;

    BeliefDistribution(bool empty = false);

    Parameters::NumType mean();
    Parameters::NumType variance();

    bool empty();

    void storeMeanVariance(double mean, double variance);

    // prevent future updates of the mean
    void lockMean();

    bool isBeliefValid() const;

    void updateBelief(Parameters::NumType a, Parameters::NumType b);

    bool operator==(const BeliefDistribution& rhs) const;

    /**
     * Stringifies PDF particles for debugging.
     */
    const std::string str() const;

    void reset();

private:
    Particles pdf;
    bool _recomputeMean, _recomputeVariance;
    bool _useStored;
    bool _meanLocked;
    bool _empty;
    Parameters::NumType _constMean() const;
    Parameters::NumType _constVariance() const;
};

// The problem: OcTreeDataNode does not provide a virtual destructor.
// To avoid memory leaks, we use a smart pointer here.
typedef octomap::OcTreeDataNode<BeliefDistribution> BeliefVoxel;