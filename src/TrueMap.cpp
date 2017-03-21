#include <stdlib.h>
#include <array>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <ros/console.h>

#include "../include/TrueMap.h"
#include "../include/PointCloud.h"

TrueMap::TrueMap() : octomap::OcTree(Parameters::voxelSize), QVoxelMap(this)
{
}

TrueMap TrueMap::generate(unsigned int seed)
{
    srand(seed);

#if defined(PLANNER_2D_TEST)
    auto obstacles = std::vector<Rectangle> {
            Rectangle(-1.0, -1.0, -0.1, -0.1),
            Rectangle(-1.0,  0.2, -0.1,  1.0)
    };
    return _generateFromObstacles(obstacles);
#else
    TrueMap map;
    auto center = Parameters::Vec3Type(Parameters::xCenter, Parameters::yCenter, Parameters::zCenter);
    // XXX note the <= instead of < here compared to LogOddsMap and BeliefMap
    for (unsigned int x = 0; x <= Parameters::voxelsPerDimensionX; ++x)
    {
        for (unsigned int y = 0; y <= Parameters::voxelsPerDimensionY; ++y)
        {
            for (unsigned int z = 0; z <= Parameters::voxelsPerDimensionZ; ++z)
            {
                octomap::point3d point(
                        Parameters::xMin + x * Parameters::voxelSize,
                        Parameters::yMin + y * Parameters::voxelSize,
                        Parameters::zMin + z * Parameters::voxelSize);
                // cells around center (where the robot is) are free
                if (center.distance(point) <= Parameters::freeRadiusAroundCenter)
                {
                    map.updateNode(point, false);
                }
                else
                {
                    bool occupied = rand() % 3 == 0; // 1/3 occupied
                    map.updateNode(point, occupied);
                }
            }
        }
    }

    map.updateSubscribers();

    ROS_INFO("True map has %d nodes in total.", (int)map.calcNumNodes());
    ROS_INFO("Voxels per dimension: %d x %d x %d (%d in total)",
             (int)Parameters::voxelsPerDimensionX,
             (int)Parameters::voxelsPerDimensionY,
             (int)Parameters::voxelsPerDimensionZ,
             (int)Parameters::voxelsTotal);
    map.calcMinMax();
    ROS_INFO("True map range: (%.2f %.2f %.2f) to (%.2f %.2f %.2f)",
             map.min_value[0], map.min_value[1], map.min_value[2],
             map.max_value[0], map.max_value[1], map.max_value[2]);
    return map;
#endif
}

void TrueMap::shuffle()
{
    auto center = Parameters::Vec3Type(Parameters::xCenter, Parameters::yCenter, Parameters::zCenter);
    // XXX note the <= instead of < here compared to LogOddsMap and BeliefMap
    for (unsigned int x = 0; x <= Parameters::voxelsPerDimensionX; ++x)
    {
        for (unsigned int y = 0; y <= Parameters::voxelsPerDimensionY; ++y)
        {
            for (unsigned int z = 0; z <= Parameters::voxelsPerDimensionZ; ++z)
            {
                octomap::point3d point(
                        Parameters::xMin + x * Parameters::voxelSize,
                        Parameters::yMin + y * Parameters::voxelSize,
                        Parameters::zMin + z * Parameters::voxelSize);
                // cells around center (where the robot is) are free
                if (center.distance(point) <= Parameters::freeRadiusAroundCenter)
                {
                    updateNode(point, false);
                }
                else
                {
                    bool occupied = rand() % 3 == 0; // 1/3 occupied
                    updateNode(point, occupied);
                }
            }
        }
    }
    updateSubscribers();
}

TrueMap TrueMap::generateFromPointCloud(std::string filename)
{
    TrueMap map;

    // initialize all cells to be free
    ROS_INFO("Setting all TrueMap cells to free.");
    auto center = Parameters::Vec3Type(Parameters::xCenter, Parameters::yCenter, Parameters::zCenter);
    // XXX note the <= instead of < here compared to LogOddsMap and BeliefMap
    for (unsigned int x = 0; x <= Parameters::voxelsPerDimensionX; ++x)
    {
        for (unsigned int y = 0; y <= Parameters::voxelsPerDimensionY; ++y)
        {
            for (unsigned int z = 0; z <= Parameters::voxelsPerDimensionZ; ++z)
            {
                octomap::point3d point(
                        Parameters::xMin + x * Parameters::voxelSize,
                        Parameters::yMin + y * Parameters::voxelSize,
                        Parameters::zMin + z * Parameters::voxelSize);
                map.updateNode(point, false);
            }
        }
    }

    PointCloud cloud;
    cloud.loadPly(filename);
    ROS_INFO("Updating TrueMap from point cloud ...");
    const double rescale = Parameters::voxelSize * 0.5;
    for (auto &cpoint : cloud.cloud())
    {
        octomap::point3d point(
                (cpoint.x / rescale) * rescale,
                (cpoint.y / rescale) * rescale,
                (cpoint.z / rescale) * rescale);
        map.updateNode(point, true);
    }

    ROS_INFO("Voxels per dimension: %d x %d x %d (%d in total)",
             (int)Parameters::voxelsPerDimensionX,
             (int)Parameters::voxelsPerDimensionY,
             (int)Parameters::voxelsPerDimensionZ,
             (int)Parameters::voxelsTotal);

    return map;
}

TrueMap TrueMap::generateCorridor()
{
    auto obstacles = std::vector<Rectangle> {
            Rectangle(-1.0, -1.0,  0.1, -0.6), // A
            Rectangle(-1.0, -0.6, -0.6,  0.6), // B
            Rectangle(-1.0,  0.6,  0.1,  1.0), // C
            Rectangle( 0.6, -1.0,  1.0,  1.0), // D
//            Rectangle(-0.1, -0.1,  0.6,  0.1), // E
            Rectangle(-1.0, -1.0,  1.0, -0.9),
            Rectangle(-1.0,  0.9,  1.0,  1.0)
    };
    return _generateFromObstacles(obstacles);
}

TrueMap TrueMap::_generateFromObstacles(const std::vector<TrueMap::Rectangle> &obstacles)
{
    TrueMap map;
    int occupied = 0;
    for (unsigned int x = 0; x < Parameters::voxelsPerDimensionX; ++x)
    {
        for (unsigned int y = 0; y < Parameters::voxelsPerDimensionY; ++y)
        {
            for (unsigned int z = 0; z < Parameters::voxelsPerDimensionZ; ++z)
            {
                octomap::point3d point(
                        (float) (Parameters::xMin + x * Parameters::voxelSize),
                        (float) (Parameters::yMin + y * Parameters::voxelSize),
                        (float) (Parameters::zMin + z * Parameters::voxelSize));

                map.updateNode(point, false);
                for (auto &obstacle: obstacles)
                {
                    if (obstacle.contains(point.x(), point.y()))
                    {
                        map.updateNode(point, true);
                        ++occupied;
                        break;
                    }
                }
            }
        }
    }
    map.updateSubscribers();

    ROS_INFO("True map has %d nodes in total.", (int)map.calcNumNodes());
    ROS_INFO("Voxels per dimension: %d x %d x %d (%d in total)",
             (int)Parameters::voxelsPerDimensionX,
             (int)Parameters::voxelsPerDimensionY,
             (int)Parameters::voxelsPerDimensionZ,
             (int)Parameters::voxelsTotal);
    ROS_INFO("%d voxels are occupied.", occupied);
    map.calcMinMax();
    ROS_INFO("True map range: (%.2f %.2f %.2f) to (%.2f %.2f %.2f)",
             map.min_value[0], map.min_value[1], map.min_value[2],
             map.max_value[0], map.max_value[1], map.max_value[2]);
    return map;
}

TrueMap TrueMap::generateCorridor2()
{
    auto obstacles = std::vector<Rectangle> {
            Rectangle(-1.0, -1.0, -0.9,  1.0),
            Rectangle(-0.9, -1.0, -0.7, -0.2), Rectangle(-0.9, -0.1, -0.4,  1.0),
            Rectangle(-0.9, -1.0, -0.4, -0.7), Rectangle(-0.9, -0.1, -0.7,  1.0),
    };
    return _generateFromObstacles(obstacles);
}
