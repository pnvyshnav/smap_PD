#pragma once

#include <array>
#include <algorithm>
#include <cmath>

#include <octomap/OcTreeDataNode.h>

#include "parameters.hpp"

class Belief
{
public:
    typedef std::array<Parameters::NumType, Parameters::numParticles> Particles;
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

typedef octomap::OcTreeDataNode<Belief> BeliefVoxel;
