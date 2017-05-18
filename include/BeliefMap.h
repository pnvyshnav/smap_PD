#pragma once

#include <array>
#include <valarray>
#include <algorithm>
#include <cmath>

#include <octomap/OcTreeDataNode.h>
#include <octomap/OcTreeBase.h>

#include "Parameters.hpp"
#include "BeliefVoxel.h"
#include "StatisticsMap.hpp"
#include "Observation.hpp"
#include "Observable.hpp"
#include "TrueMap.h"
#include "InverseCauseModel.h"

class InverseCauseModel;
class BeliefMap
        : public octomap::OcTreeBaseImpl<BeliefVoxel, octomap::AbstractOcTree>,
          public StatisticsMap<BeliefVoxel, octomap::AbstractOcTree>,
          public Observable
{
public:
    BeliefMap();
    BeliefMap(const BeliefMap &map);

    BeliefMap *create() const;

    BeliefMap copy() const;

    BeliefMap &operator=(const BeliefMap &map);

    Belief belief(const octomap::OcTreeKey &key) const;

    BeliefVoxel *updateNode(const octomap::point3d& position, const Belief &belief);
    BeliefVoxel *updateNode(const octomap::OcTreeKey& key, const Belief &belief);

    std::valarray<Parameters::NumType> bouncingProbabilitiesOnRay(
            const octomap::KeyRay &ray) const;

    std::valarray<Parameters::NumType> reachingProbabilitiesOnRay(
            const octomap::KeyRay &ray,
            const std::valarray<Parameters::NumType> &bouncingProbabilities) const;

    // TODO remove trueMap argument
    bool update(const Observation &observation, const TrueMap &trueMap);

    void reset();

    std::vector<QBeliefVoxel> updatedVoxels() const
    {
        return _lastUpdatedVoxels;
    }

    double getVoxelMean(QBeliefVoxel &voxel) const
    {
        if (voxel.type != GEOMETRY_VOXEL)
            return Parameters::priorMean;
        return voxel.node()->getValue().mean();
    }

    double getVoxelStd(QBeliefVoxel &voxel) const
    {
        if (voxel.type != GEOMETRY_VOXEL)
            return Parameters::priorStd;
        return std::sqrt(voxel.node()->getValue().variance());
    }

    std::string mapType() const
    {
        return "Belief";
    }

    InverseCauseModel icm() const
    {
        return _icm;
    }

protected:
    BeliefVoxel *updateNodeRecurs(BeliefVoxel *node, bool node_just_created,
                                  const octomap::OcTreeKey &key,
                                  unsigned int depth, const Belief &belief);

private:
    std::vector<QBeliefVoxel> _lastUpdatedVoxels;

    InverseCauseModel _icm;

    void _expandNode(BeliefVoxel *node);
    BeliefVoxel *_createNodeChild(BeliefVoxel *node, unsigned int childIdx);
};
