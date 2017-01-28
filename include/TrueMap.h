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
    static TrueMap generateFromPointCloud(std::string filename);

    /**
     * Alters the map with a new random map.
     */
    void shuffle();

    static inline bool insideMap(const Parameters::Vec3Type &point)
    {
        return point.x() >= Parameters::xMin && point.x() <= Parameters::xMax
               && point.y() >= Parameters::yMin && point.y() <= Parameters::yMax
               && point.z() >= Parameters::zMin && point.z() <= Parameters::zMax;
    }

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

