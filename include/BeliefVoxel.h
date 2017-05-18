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
    typedef std::valarray<Parameters::NumType> Particles;

    Belief(bool empty = false);

    bool empty();

    Parameters::NumType mean();
    Parameters::NumType variance();

    void storeMeanVariance(double mean, double variance);

    // prevent future updates of the mean
    void lockMean();

    bool isBeliefValid() const;

    void updateBelief(Parameters::NumType a, Parameters::NumType b);

    bool operator==(const Belief& rhs) const;

    /**
     * Stringifies PDF particles for debugging.
     */
    const std::string str() const;

    void reset();

private:
    Particles pdf;
    bool _recomputeMean, _recomputeVariance;
    Parameters::NumType _mean, _variance;
    bool _useStored;
    bool _meanLocked;
    bool _empty;
    Parameters::NumType _constMean() const;
    Parameters::NumType _constVariance() const;
};

// The problem: OcTreeDataNode does not provide a virtual destructor.
// To avoid memory leaks, we use a smart pointer here.
typedef octomap::OcTreeDataNode<Belief> BeliefVoxel;