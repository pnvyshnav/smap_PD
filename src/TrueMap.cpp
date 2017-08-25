#include <stdlib.h>
#include <array>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <ros/console.h>

#include "../include/TrueMap.h"
#include "../include/PointCloud.h"
#include "../include/Drone.h"


octomap::OcTree _helpMap(Parameters::voxelSize);

TrueMap::TrueMap() : octomap::OcTree(Parameters::voxelSize), QVoxelMap(this), _empty(true)
{
}

TrueMap TrueMap::generate(unsigned int seed)
{
    srand(seed);
    TrueMap map;
    map._empty = false;

#if defined(PLANNER_2D_TEST)
    struct Rectangle
    {
        double x1, x2;
        double y1, y2;

        Rectangle(double _x1, double _y1, double _x2, double _y2)
                : x1(_x1), x2(_x2), y1(_y1), y2(_y2)
        {}

        bool contains(double x, double y)
        {
            return x >= x1 && x <= x2 && y >= y1 && y <= y2;
        }
    };

    std::vector<Rectangle> obstacles = std::vector<Rectangle> {
            Rectangle(-1.0, -1.0, -0.1, -0.1),
            Rectangle(-1.0,  0.2, -0.1,  1.0)
    };

    for (unsigned int x = 0; x < Parameters::voxelsPerDimensionX(); ++x)
    {
        for (unsigned int y = 0; y < Parameters::voxelsPerDimensionY(); ++y)
        {
            for (unsigned int z = 0; z < Parameters::voxelsPerDimensionZ(); ++z)
            {
                octomap::point3d point(
                        (float) (Parameters::xMin + x * Parameters::voxelSize),
                        (float) (Parameters::yMin + y * Parameters::voxelSize),
                        (float) (Parameters::zMin + z * Parameters::voxelSize));

                for (auto &obstacle: obstacles)
                {
                    map.updateNode(point, obstacle.contains(point.x(), point.y()));
                }
            }
        }
    }
#else
    auto center = Parameters::center();
    // XXX note the <= instead of < here compared to LogOddsMap and BeliefMap
    for (unsigned int x = 0; x <= Parameters::voxelsPerDimensionX(); ++x)
    {
        for (unsigned int y = 0; y <= Parameters::voxelsPerDimensionY(); ++y)
        {
            for (unsigned int z = 0; z <= Parameters::voxelsPerDimensionZ(); ++z)
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
#endif

    map.updateSubscribers();

    ROS_INFO("True map has %d nodes in total.", (int)map.calcNumNodes());
    ROS_INFO("Voxels per dimension: %d x %d x %d (%d in total)",
             (int)Parameters::voxelsPerDimensionX(),
             (int)Parameters::voxelsPerDimensionY(),
             (int)Parameters::voxelsPerDimensionZ(),
             (int)Parameters::voxelsTotal());
    map.calcMinMax();
    ROS_INFO("True map range: (%.2f %.2f %.2f) to (%.2f %.2f %.2f)",
             map.min_value[0], map.min_value[1], map.min_value[2],
             map.max_value[0], map.max_value[1], map.max_value[2]);
    return map;
}

void TrueMap::shuffle()
{
    _empty = false;
    auto center = Parameters::center();
    // XXX note the <= instead of < here compared to LogOddsMap and BeliefMap
    for (unsigned int x = 0; x <= Parameters::voxelsPerDimensionX(); ++x)
    {
        for (unsigned int y = 0; y <= Parameters::voxelsPerDimensionY(); ++y)
        {
            for (unsigned int z = 0; z <= Parameters::voxelsPerDimensionZ(); ++z)
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
    map._empty = false;

    // initialize all cells to be free
    ROS_INFO("Setting all TrueMap cells to free.");
    auto center = Parameters::center();
    // XXX note the <= instead of < here compared to LogOddsMap and BeliefMap
    for (unsigned int x = 0; x <= Parameters::voxelsPerDimensionX(); ++x)
    {
        for (unsigned int y = 0; y <= Parameters::voxelsPerDimensionY(); ++y)
        {
            for (unsigned int z = 0; z <= Parameters::voxelsPerDimensionZ(); ++z)
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
             (int)Parameters::voxelsPerDimensionX(),
             (int)Parameters::voxelsPerDimensionY(),
             (int)Parameters::voxelsPerDimensionZ(),
             (int)Parameters::voxelsTotal());

    return map;
}

octomap::OcTreeKey TrueMap::coordToKey(const octomap::point3d &position)
{
    return _helpMap.coordToKey(position);
}

octomap::point3d TrueMap::keyToCoord(octomap::OcTreeKey key)
{
    return _helpMap.keyToCoord(key);
}

TrueMap TrueMap::generateCorridor()
{
    std::vector<Box> obstacles;
    obstacles.push_back(Box(0.00, 0.00, 0.00, 0.00));
    obstacles.push_back(Box(-1.00, -0.05, 0.74, 0.06));
    obstacles.push_back(Box(-0.26, 0.73, 1.22, 0.06));
    obstacles.push_back(Box(-0.26, -0.99, 0.36, 0.06));
    obstacles.push_back(Box(-1.00, -0.51, 0.74, 0.06));
    obstacles.push_back(Box(0.10, -0.51, 0.86, 0.06));
    obstacles.push_back(Box(0.32, 0.07, 0.06, 0.36));
    obstacles.push_back(Box(0.38, 0.07, 0.28, 0.06));
    obstacles.push_back(Box(0.50, 0.25, 0.16, 0.18));
    obstacles.push_back(Box(0.90, -0.45, 0.06, 1.18));
    obstacles.push_back(Box(-0.92, -0.19, 0.16, 0.06));
    obstacles.push_back(Box(-0.70, -0.19, 0.16, 0.06));
    obstacles.push_back(Box(-0.48, -0.19, 0.16, 0.06));
    obstacles.push_back(Box(0.22, -0.39, 0.16, 0.06));
    obstacles.push_back(Box(0.44, -0.39, 0.16, 0.06));
    obstacles.push_back(Box(0.66, -0.39, 0.16, 0.06));
    obstacles.push_back(Box(0.50, 0.63, 0.16, 0.06));
    obstacles.push_back(Box(0.14, 0.45, 0.12, 0.04));
    obstacles.push_back(Box(0.26, 0.63, 0.12, 0.04));
    obstacles.push_back(Box(-0.22, 0.47, 0.08, 0.16));
    obstacles.push_back(Box(0.86, 0.37, 0.02, 0.06));
    obstacles.push_back(Box(0.86, -0.05, 0.02, 0.06));
    obstacles.push_back(Box(-0.22, 0.27, 0.08, 0.16));
    obstacles.push_back(Box(-0.32, 0.01, 0.06, 0.78));
    obstacles.push_back(Box(-0.32, -0.99, 0.06, 0.48));
    obstacles.push_back(Box(0.10, -0.99, 0.06, 0.48));
    obstacles.push_back(Box(0.10, -0.05, 0.56, 0.06));
    obstacles.push_back(Box(0.60, 0.01, 0.06, 0.06));
    obstacles.push_back(Box(0.10, -0.05, 0.06, 0.48));
    obstacles.push_back(Box(0.16, 0.37, 0.16, 0.06));
    return _generateFromObstacles(obstacles);
}

TrueMap TrueMap::_generateFromObstacles(const std::vector<Box> &obstacles)
{
    TrueMap map;
    map._empty = false;
    int occupied = 0;
    for (unsigned int x = 0; x < Parameters::voxelsPerDimensionX(); ++x)
    {
        for (unsigned int y = 0; y < Parameters::voxelsPerDimensionY(); ++y)
        {
            for (unsigned int z = 0; z < Parameters::voxelsPerDimensionZ(); ++z)
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
             (int)Parameters::voxelsPerDimensionX(),
             (int)Parameters::voxelsPerDimensionY(),
             (int)Parameters::voxelsPerDimensionZ(),
             (int)Parameters::voxelsTotal());
    ROS_INFO("%d voxels are occupied.", occupied);
    map.calcMinMax();
    ROS_INFO("True map range: (%.2f %.2f %.2f) to (%.2f %.2f %.2f)",
             map.min_value[0], map.min_value[1], map.min_value[2],
             map.max_value[0], map.max_value[1], map.max_value[2]);
    return map;
}

std::vector<size_t> pointCounts;

void handleObservation(TrueMap *trueMap, const Observation &observation)
{
    const octomap::Pointcloud cloud = observation.pointcloud();
    octomap::point3d lowerBound, upperBound;
    cloud.calcBBX(lowerBound, upperBound);
//    ROS_INFO("Bounding box of %i measurements: (%.2f, %.2f, %.2f) -- (%.2f, %.2f, %.2f)",
//             (int) cloud.size(),
//             lowerBound.x(), lowerBound.y(), lowerBound.z(),
//             upperBound.x(), upperBound.y(), upperBound.z()
//    );
//    const octomap::point3d origin;
//    bool lazyEval = false;
//    trueMap->insertPointCloud(cloud, origin, Parameters::sensorRange, lazyEval);
    pointCounts.push_back(cloud.size());
    for (auto sensorPoint : observation.discretized())
    {
        trueMap->updateNode(sensorPoint.position, sensorPoint.occupied);
    }
}

TrueMap TrueMap::generateFromCarmen(std::string filename, std::string messageName,
                                    bool oldFormat, const unsigned int everyNth)
{
    pointCounts.clear();
    TrueMap map;
    map._empty = false;
    Drone drone;
    drone.registerObserver(std::bind(handleObservation, &map, std::placeholders::_1));
    drone.runCarmenFile(filename, messageName, oldFormat, everyNth);

    ROS_INFO("True map has %d nodes in total.", (int)map.calcNumNodes());
    ROS_INFO("Voxels per dimension: %d x %d x %d (%d in total)",
             (int)Parameters::voxelsPerDimensionX(),
             (int)Parameters::voxelsPerDimensionY(),
             (int)Parameters::voxelsPerDimensionZ(),
             (int)Parameters::voxelsTotal());
    size_t totalPointCounts = 0;
    for (auto c : pointCounts)
        totalPointCounts += c;
    ROS_INFO("%d points in total, %.2f points per cloud on average were registered.",
             (int)totalPointCounts, (float)(totalPointCounts*1.f/pointCounts.size()));
    map.calcMinMax();
    ROS_INFO("True map range: (%.2f %.2f %.2f) to (%.2f %.2f %.2f)",
             map.min_value[0], map.min_value[1], map.min_value[2],
             map.max_value[0], map.max_value[1], map.max_value[2]);

    return map;
}

TrueMap &TrueMap::operator=(const TrueMap &map)
{
    for (unsigned int x = 0; x < Parameters::voxelsPerDimensionX(); ++x)
    {
        for (unsigned int y = 0; y < Parameters::voxelsPerDimensionY(); ++y)
        {
            for (unsigned int z = 0; z < Parameters::voxelsPerDimensionZ(); ++z)
            {
                octomap::point3d point(
                        Parameters::xMin + x * Parameters::voxelSize,
                        Parameters::yMin + y * Parameters::voxelSize,
                        Parameters::zMin + z * Parameters::voxelSize);

                float logOdds = map.search(point)->getLogOdds();
                setNodeValue(point, logOdds);
            }
        }
    }
    return *this;
}
