#pragma once

#include <octomap/OcTree.h>
#include "Parameters.h"
#include "QVoxel.hpp"
#include "Observable.hpp"
#include "Box.hpp"


class TrueMap
        : public octomap::OcTree,
          public QVoxelMap<octomap::OcTreeNode, octomap::AbstractOccupancyOcTree>,
          public Observable
{
public:
    TrueMap();

    static TrueMap generate(unsigned int seed = (unsigned int) time(nullptr));
    static TrueMap generateFromPointCloud(std::string filename);
    static TrueMap generateFromCarmen(std::string filename, std::string messageName = "FLASER",
                                      bool oldFormat = true, unsigned int everyNth = 1);
    static TrueMap generateCorridor();

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
        if (!voxel.node())
            return Parameters::priorMean;
        return std::round(voxel.node()->getOccupancy());
    }

    double getVoxelStd(QTrueVoxel &voxel) const
    {
        return 0;
    }

    static octomap::OcTreeKey coordToKey(const octomap::point3d &position);
    static octomap::point3d keyToCoord(octomap::OcTreeKey key);

    TrueMap &operator=(const TrueMap &map);

    inline bool empty() const
    {
        return _empty;
    }

private:
    static TrueMap _generateFromObstacles(const std::vector<Box> &obstacles);
    bool _empty;
};

