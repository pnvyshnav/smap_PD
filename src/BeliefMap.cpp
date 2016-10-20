#include "../include/BeliefMap.h"

#include <array>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <cassert>
#include <memory>

#include <octomap/AbstractOcTree.h>

#include "../include/Parameters.hpp"
#include "../include/Sensor.h"

class registerTreeType;

std::string keyToStr(octomap::OcTreeKey key)
{
    std::stringstream ss;
    ss << "(" << key[0] << ", " << key[1] << ", " << key[2] << ")";
    return ss.str();
}

BeliefMap::BeliefMap()
        : octomap::OcTreeBaseImpl<BeliefVoxel, octomap::AbstractOcTree>(Parameters::voxelSize),
          QVoxelMap(this)
{
    octomap::AbstractOcTree::registerTreeType(this);
    init();

    for (Parameters::NumType x = Parameters::xMin; x <= Parameters::xMax; x += Parameters::voxelSize)
    {
        for (Parameters::NumType y = Parameters::yMin; y <= Parameters::yMax; y += Parameters::voxelSize)
        {
            for (Parameters::NumType z = Parameters::zMin; z <= Parameters::zMax; z += Parameters::voxelSize)
            {
                octomap::point3d point(x, y, z);
                Belief belief;
                this->updateNode(point, belief);
            }
        }
    }

    ROS_INFO("Belief map has %d nodes in total.", (int)calcNumNodes());
    calcMinMax();
    ROS_INFO("Belief map range: (%.2f %.2f %.2f) to (%.2f %.2f %.2f)",
             min_value[0], min_value[1], min_value[2],
             max_value[0], max_value[1], max_value[2]);
}

std::string BeliefMap::getTreeType() const
{
    return "BeliefMap";
}

BeliefMap *BeliefMap::create() const
{
    return new BeliefMap();
}

Belief *BeliefMap::belief(const octomap::OcTreeKey &key) const
{
    BeliefVoxel *voxel = search(key);
    if (voxel != NULL)
        return voxel->getValue().get();
    return NULL;
}

BeliefVoxel *BeliefMap::updateNode(const octomap::point3d& position, const Belief &belief)
{
    octomap::OcTreeKey key;
    if (!coordToKeyChecked(position, key))
        return NULL;
    return updateNode(key, belief);
}

BeliefVoxel *BeliefMap::updateNode(const octomap::OcTreeKey& key, const Belief &belief)
{
    BeliefVoxel *leaf = this->search(key);
    if (leaf != NULL)
    {
        leaf->setValue(std::make_shared<Belief>(belief));
        return leaf;
    }

    bool createdRoot = false;
    if (this->root == NULL){
        this->root = new BeliefVoxel();
        this->tree_size++;
        createdRoot = true;
    }

    return _updateNodeRecurs(this->root, createdRoot, key, 0, belief);
}

BeliefVoxel *BeliefMap::_updateNodeRecurs(BeliefVoxel* node, bool node_just_created, const octomap::OcTreeKey& key,
                                          unsigned int depth, const Belief &belief)
{
    bool created_node = false;

    assert(node);

    // follow down to last level
    if (depth < this->tree_depth)
    {
        unsigned int pos = computeChildIdx(key, this->tree_depth - 1 - depth);
        if (!node->childExists(pos))
        {
            // child does not exist, but maybe it's a pruned node?
            if (!node->hasChildren() && !node_just_created)
            {
                // current node does not have children AND it is not a new node
                // -> expand pruned node
                node->expandNode();
            }
            else
            {
                // not a pruned node, create requested child
                node->createChild(pos);
                created_node = true;
            }
        }

        BeliefVoxel *result = _updateNodeRecurs(node->getChild(pos), created_node, key, depth + 1, belief);
        // prune node if possible, otherwise set own probability
        // note: combining both did not lead to a speedup!
        if (node->pruneNode())
        {
            // return pointer to current parent (pruned), the just updated node no longer exists
            result = node;
        }

        return result;
    }
    else
    {
        // at last level, update node, end of recursion
        node->setValue(std::make_shared<Belief>(belief));
        return node;
    }
}

std::valarray<Parameters::NumType> BeliefMap::bouncingProbabilitiesOnRay(const octomap::KeyRay &ray) const
{
    std::valarray<Parameters::NumType> bouncingProbabilities(ray.ray.size());
    for (unsigned int i = 0; i < ray.ray.size(); ++i)
    {
        auto *b = belief(ray.ray[i]);
        if (b == NULL)
            bouncingProbabilities[i] = 0; // correct error handling?
        else
            bouncingProbabilities[i] = b->mean();
    }
    return bouncingProbabilities;
}

std::valarray<Parameters::NumType> BeliefMap::reachingProbabilitiesOnRay(const octomap::KeyRay &ray,
                                                                         const std::valarray<Parameters::NumType> &bouncingProbabilities) const
{
    assert(ray.ray.size() == bouncingProbabilities.size());
    std::valarray<Parameters::NumType> reachingProbabilities(ray.ray.size());
    reachingProbabilities[0] = (Parameters::NumType) (1. - Parameters::spuriousMeasurementProbability);
    for (unsigned int i = 1; i < ray.ray.size(); ++i)
    {
        reachingProbabilities[i] = (Parameters::NumType) (reachingProbabilities[i - 1] * (1. - bouncingProbabilities[i - 1]));
    }
    return reachingProbabilities;
}

bool BeliefMap::update(const Observation &observation, TrueMap &trueMap)
{
    for (auto &measurement : observation.measurements())
    {
        auto icm = measurement.sensor->computeInverseCauseModel(measurement, trueMap, *this);
        Parameters::NumType prBeforeVoxel = 0;
        Parameters::NumType prAfterVoxel = icm.posteriorOnRay.sum();
        Parameters::NumType prOnVoxel;
        for (unsigned int i = 0; i < icm.voxelKeys.size(); ++i)
        {
            prOnVoxel = icm.posteriorOnRay[i];

            BeliefVoxel *beliefVoxel = search(icm.voxelKeys[i]);
            if (beliefVoxel == NULL)
            {
                ROS_ERROR("Belief voxel for given voxel key on ray could not be found.");
                return false;
            }

            if (!beliefVoxel->getValue().get()->isBeliefValid())
            {
                ROS_ERROR("Belief voxel has invalid PDF before updating with measurement.");
                return false;
            }

            Parameters::NumType mean = beliefVoxel->getValue().get()->mean();
            Parameters::NumType a = (Parameters::NumType) ((1. - mean) * prOnVoxel - mean * prAfterVoxel);
            Parameters::NumType b = (Parameters::NumType) (
                    mean * (1. - mean) * (Parameters::spuriousMeasurementProbability + prBeforeVoxel)
                    + mean * prAfterVoxel);
            beliefVoxel->getValue().get()->updateBelief(a, b);

            if (!beliefVoxel->getValue().get()->isBeliefValid())
            {
                ROS_ERROR("Belief voxel has invalid PDF after updating with measurement.");
                return false;
            }

            prBeforeVoxel += prOnVoxel;
            prAfterVoxel -= prOnVoxel;
        }
    }

    return true;
}