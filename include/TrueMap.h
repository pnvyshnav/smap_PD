#pragma once

#include <octomap/OcTree.h>
#include "Parameters.hpp"
#include "QVoxel.hpp"

class TrueMap : public octomap::OcTree, public QVoxelMap<octomap::OcTreeNode, octomap::AbstractOccupancyOcTree>
{
public:
	static TrueMap generate(unsigned int seed = 1);

private:
    TrueMap();
};

