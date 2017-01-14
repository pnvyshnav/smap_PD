#pragma once

#include <octomap/OcTree.h>
#include "Parameters.hpp"
#include "QVoxel.hpp"
#include "Observable.hpp"

class TrueMap
        : public octomap::OcTree,
          public QVoxelMap<octomap::OcTreeNode, octomap::AbstractOccupancyOcTree>,
          public Observable
{
public:
    static TrueMap generate(unsigned int seed = (unsigned int) time(NULL));

    static bool insideMap(const Parameters::Vec3Type &point);

    double getVoxelMean(QTrueVoxel &voxel) const
    {
        return std::round(voxel.node()->getOccupancy());
    }

    double getVoxelStd(QTrueVoxel &voxel) const
    {
        return 0;
    }

private:
    TrueMap();
};

