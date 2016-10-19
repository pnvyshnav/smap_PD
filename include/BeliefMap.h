#pragma once

#include <array>
#include <valarray>
#include <algorithm>
#include <cmath>

#include <octomap/OcTreeDataNode.h>
#include <octomap/OcTreeBase.h>

#include "Parameters.hpp"
#include "BeliefVoxel.h"
#include "QVoxel.hpp"

class BeliefMap : public octomap::OcTreeBaseImpl<BeliefVoxel, octomap::AbstractOcTree>, public QVoxelMap<BeliefVoxel, octomap::AbstractOcTree>
{
public:
    BeliefMap();

    std::string getTreeType() const;

    BeliefMap *create() const;

    Belief *belief(const octomap::OcTreeKey &key) const;

    std::valarray<Parameters::NumType> bouncingProbabilitiesOnRay(const octomap::KeyRay &ray) const;

    std::valarray<Parameters::NumType> reachingProbabilitiesOnRay(const octomap::KeyRay &ray,
                                                                  const std::valarray<Parameters::NumType> &bouncingProbabilities) const;

private:
};

