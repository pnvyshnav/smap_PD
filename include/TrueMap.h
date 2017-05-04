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
    static TrueMap generateCorridor();
    static TrueMap generateCorridor2();
    static TrueMap generateRandomCorridor(int radius = 2, int branches = 20,
                                          unsigned int samplingWidth = 32, unsigned int samplingHeight = 32,
                                          float difficulty = 0.5f,
                                          unsigned int seed = (unsigned int) time(NULL));

    /**
     * Alters the map with a new random map.
     */
    void shuffle();
    void shuffleCorridor(int radius = 2, int branches = 20,
                         float difficulty = 0.5f,
                         unsigned int seed = (unsigned int) time(NULL));

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

    TrueMap operator=(TrueMap &map);
    TrueMap operator=(TrueMap map);

    Parameters::Vec3Type start() const;
    Parameters::Vec3Type goal() const;

private:
    TrueMap();

    struct Rectangle
    {
        double x1, x2;
        double y1, y2;

        Rectangle(double _x1, double _y1, double _x2, double _y2)
                : x1(_x1), x2(_x2), y1(_y1), y2(_y2)
        {}

        bool contains(double x, double y) const
        {
            return x >= x1 && x <= x2 && y >= y1 && y <= y2;
        }
    };

    static TrueMap _generateFromObstacles(const std::vector<TrueMap::Rectangle> &obstacles);

    Parameters::Vec3Type _start, _goal;
};

