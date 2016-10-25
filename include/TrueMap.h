#pragma once

#include <octomap/OcTree.h>
#include "Parameters.hpp"
#include "QVoxel.hpp"
#include "Visualizable.hpp"

class TrueMap
        : public octomap::OcTree,
          public QVoxelMap<octomap::OcTreeNode, octomap::AbstractOccupancyOcTree>,
          public Visualizable
{
public:
    static TrueMap generate(unsigned int seed = (unsigned int) time(NULL));

private:
    TrueMap();
};

