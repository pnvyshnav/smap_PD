#include "../include/BeliefMap.h"

#include <array>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <cassert>

#include "../include/Parameters.hpp"
#include <octomap/AbstractOcTree.h>
#include <memory>

class registerTreeType;

std::string keyToStr(octomap::OcTreeKey key)
{
    std::stringstream ss;
    ss << "(" << key[0] << ", " << key[1] << ", " << key[2] << ")";
    return ss.str();
}

BeliefMap::BeliefMap() : octomap::OcTreeBaseImpl<BeliefVoxel, octomap::AbstractOcTree>(
                             Parameters::voxelSize, Parameters::maxDepth, Parameters::voxelsPerDimension),
                         QVoxelMap(this)
{
    init();
    //tree_center = octomap::point3d(3, 2, 1);
    Belief b;
    root = new BeliefVoxel();
    root->setValue(std::make_shared<Belief>(b));
    root->expandNode();
    this->expand();
    octomap::AbstractOcTree::registerTreeType(this);

    calcMinMax();

    std::cout << "vox_size:  " << Parameters::voxelSize << std::endl;
    std::cout << "max_depth: " << Parameters::maxDepth << std::endl;
    std::cout << "vox_p_dim: " << Parameters::voxelsPerDimension << std::endl;
    std::cout << "min_coord: " << octomap::point3d(min_value[0], min_value[1], min_value[2]) << std::endl;
    std::cout << "max_coord: " << octomap::point3d(max_value[0], max_value[1], max_value[2]) << std::endl;
    std::cout << "num_voxel: " << calcNumNodes() << std::endl;


    //this->computeRayKeys(octomap::point3d(), octomap::point3d(2, 4, 3));

    //tree_center = octomap::point3d(3, 2, 1);

    octomap::OcTreeKey key;
    //root->getValue().updateBelief(4.f, 5.f);
    std::cout << keyToStr(coordToKey(1.6,-1.5,-1.5)) << std::endl;
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