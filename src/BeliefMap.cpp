#include "BeliefMap.h"

#include <array>
#include <algorithm>
#include <cmath>
#include <sstream>

#include "Parameters.hpp"
#include <octomap/AbstractOcTree.h>

class registerTreeType;

std::string keyToStr(octomap::OcTreeKey key)
{
    std::stringstream ss;
    ss << "(" << key[0] << ", " << key[1] << ", " << key[2] << ")";
    return ss.str();
}

BeliefMap::BeliefMap() : octomap::OcTreeBaseImpl<BeliefVoxel, octomap::AbstractOcTree>(
                             Parameters::voxelSize, Parameters::maxDepth, Parameters::voxelsPerDimension)
{
    init();
    //tree_center = octomap::point3d(3, 2, 1);
    Belief b;
    root = new BeliefVoxel();
    root->setValue(b);
    root->expandNode();
    this->expand();
    AbstractOcTree::registerTreeType(this);

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
