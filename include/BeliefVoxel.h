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

    static const Particles particles;

    Belief();

    Parameters::NumType mean() const;
    Parameters::NumType variance() const;

    bool isBeliefValid() const;

    void updateBelief(Parameters::NumType a, Parameters::NumType b);

    bool operator== (const Belief& rhs) const;

private:
    Particles pdf;
    static Particles generateParticles();
};

// The problem: OcTreeDataNode does not provide a virtual destructor.
// To avoid memory leaks, we use a smart pointer here.
typedef octomap::OcTreeDataNode<std::shared_ptr<Belief> > BeliefVoxel;
