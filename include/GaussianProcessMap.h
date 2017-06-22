#pragma once

#include <gp/gp.h>

#include "Observation.hpp"
#include "BeliefVoxel.h"
#include "Observable.hpp"

class GaussianProcessMap : public Observable
{
public:
    GaussianProcessMap(std::string kernel = "CovSum ( CovMatern5iso, CovNoise)");

    bool update(const Observation &observation);

    Belief belief(const octomap::point3d& position);

private:
    libgp::GaussianProcess _gp;
};
