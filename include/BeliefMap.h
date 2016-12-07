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
#include "Observation.hpp"

class InverseCauseModel;
class BeliefMap
        : public octomap::OcTreeBaseImpl<BeliefVoxel, octomap::AbstractOcTree>,
          public QVoxelMap<BeliefVoxel, octomap::AbstractOcTree>,
          public Observable
{
public:
    BeliefMap();

    std::string getTreeType() const;

    BeliefMap *create() const;

    Belief *belief(const octomap::OcTreeKey &key) const;

    BeliefVoxel *updateNode(const octomap::point3d& position, const Belief &belief);
    BeliefVoxel *updateNode(const octomap::OcTreeKey& key, const Belief &belief);

    std::valarray<Parameters::NumType> bouncingProbabilitiesOnRay(
            const octomap::KeyRay &ray) const;

    std::valarray<Parameters::NumType> reachingProbabilitiesOnRay(
            const octomap::KeyRay &ray,
            const std::valarray<Parameters::NumType> &bouncingProbabilities) const;

    // TODO remove trueMap argument
    bool update(const Observation &observation, const TrueMap &trueMap);

    InverseCauseModel *icm;
    std::vector<QBeliefVoxel> lastUpdatedVoxels;

    std::vector<double> error(const TrueMap &trueMap) const;
    std::vector<double> errorLastUpdated(const TrueMap &trueMap) const;

    void reset();

private:
    BeliefVoxel *_updateNodeRecurs(BeliefVoxel* node, bool node_just_created,
                                   const octomap::OcTreeKey& key,
                                   unsigned int depth, const Belief &belief);

    void _expandNode(BeliefVoxel *node);
    BeliefVoxel *_createNodeChild(BeliefVoxel *node, unsigned int childIdx);
};
