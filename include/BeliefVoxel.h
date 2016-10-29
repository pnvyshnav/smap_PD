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

    Belief();

    Parameters::NumType mean();
    Parameters::NumType variance();

    bool isBeliefValid() const;

    void updateBelief(Parameters::NumType a, Parameters::NumType b);

    bool operator== (Belief& rhs);

private:
    Particles pdf;
    bool _recompute;
    Parameters::NumType _mean, _variance;
};

// The problem: OcTreeDataNode does not provide a virtual destructor.
// To avoid memory leaks, we use a smart pointer here.
typedef octomap::OcTreeDataNode<std::shared_ptr<Belief> > BeliefVoxel;
