#include <stdlib.h>
#include <array>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <ros/console.h>

#include "../include/TrueMap.h"
#include "../include/PointCloud.h"

/**
 * Computes indices of sorted array.
 * Src: http://stackoverflow.com/a/12399290
 * @tparam T Type of vector.
 * @param v Vector values to be sorted.
 * @return Indices of ascendingly sorted values.
 */
template <typename T>
std::vector<size_t> sort_indexes(const std::vector<T> &v)
{
    // initialize original index locations
    std::vector<size_t> idx(v.size());
    std::iota(idx.begin(), idx.end(), 0);

    // sort indexes based on comparing values in v
    std::sort(idx.begin(), idx.end(),
         [&v](size_t i1, size_t i2) {return v[i1] < v[i2];});

    return idx;
}

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
            Rectangle(-0.9, -1.0, -0.7, -0.2),
            Rectangle(-0.9, -0.1, -0.4,  1.0),
            Rectangle(-0.9, -1.0, -0.4, -0.7),
            Rectangle(-0.9, -0.1, -0.7,  1.0),
    };
    return _generateFromObstacles(obstacles);
}

void fillBlock(TrueMap *map, int x1, int x2, int y1, int y2, bool value = false,
               float xmin = Parameters::xMin,
               float ymin = Parameters::yMin,
               float zmin = Parameters::zMin)
{
    for (int x = std::max(1, std::min(x1, x2)); x <= std::min((int)Parameters::voxelsPerDimensionX-1, std::max(x1, x2)); ++x)
    {
        for (int y = std::max(1, std::min(y1, y2)); y <= std::min((int)Parameters::voxelsPerDimensionY-1, std::max(y1, y2)); ++y)
        {
            for (int z = 0; z <= Parameters::voxelsPerDimensionZ; ++z)
            {
                octomap::point3d point(
                        xmin + x * Parameters::voxelSize,
                        ymin + y * Parameters::voxelSize,
                        zmin + z * Parameters::voxelSize);
                for (int i = 0; i < 20; ++i)
                    map->updateNode(point, value);
                //ROS_INFO("Updating %i %i %i to %i", x, y, z, value);
            }
        }
    }
}

TrueMap TrueMap::generateRandomCorridor(int radius, int branches,
                                        unsigned int samplingWidth, unsigned int samplingHeight,
                                        float difficulty,
                                        unsigned int seed)
{
    srand(seed);
    TrueMap map;

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
                map.updateNode(point, true);
            }
        }
    }

    fillBlock(&map, Parameters::voxelsPerDimensionX/2 - radius, Parameters::voxelsPerDimensionX/2 + radius,
              Parameters::voxelsPerDimensionY/2 - radius, Parameters::voxelsPerDimensionY/2 + radius);

    const float xmin = Parameters::xMin + (Parameters::voxelsPerDimensionX-samplingWidth)/2.f * Parameters::voxelSize;
    const float ymin = Parameters::yMin + (Parameters::voxelsPerDimensionY-samplingHeight)/2.f * Parameters::voxelSize;
    std::vector<Parameters::Vec3Type> positions({Parameters::Vec3Type(Parameters::xCenter,
                                                                      Parameters::yCenter,
                                                                      Parameters::zCenter)});
    for (int k = 0; k < branches; ++k)
    {
        int i = (int)(rand() * 1. / RAND_MAX * samplingWidth);
        int j = (int)(rand() * 1. / RAND_MAX * samplingHeight);
        // roll the dice again if we are too close to the center (double the chances of have more expanding maps)
//        while (std::abs(i - Parameters::voxelsPerDimensionX/2) < Parameters::voxelsPerDimensionX * 0.3
//                && std::abs(j - Parameters::voxelsPerDimensionY/2) < Parameters::voxelsPerDimensionY * 0.3)
//        {
//            i = (int)(rand() * 1. / RAND_MAX * Parameters::voxelsPerDimensionX);
//            j = (int)(rand() * 1. / RAND_MAX * Parameters::voxelsPerDimensionY);
//        }
        float nx = xmin + i * Parameters::voxelSize;
        float ny = ymin + j * Parameters::voxelSize;

        // find closest vertex
        double minDistance = std::numeric_limits<double>::max();
        Parameters::Vec3Type closest;
        for (auto &pos : positions)
        {
            double d = pos.distance(Parameters::Vec3Type(nx, ny, Parameters::zCenter));
            if (d < minDistance)
            {
                minDistance = d;
                closest = pos;
            }
        }

        int mi = (int) ((closest.x() - xmin) / Parameters::voxelSize);
        int mj = (int) ((closest.y() - ymin) / Parameters::voxelSize);

        if (std::abs(nx - closest.x()) < std::abs(ny - closest.y()))
        {
            // connect vertically
            fillBlock(&map, mi - radius, mi + radius,
                      std::min(mj, j) - radius,
                      std::max(mj, j) + radius,
                      false, xmin, ymin);
            positions.push_back(Parameters::Vec3Type(closest.x(), ny, Parameters::zCenter));
        }
        else
        {
            // connect horizontally
            fillBlock(&map, std::min(mi, i) - radius, std::max(mi, i) + radius,
                      mj - radius, mj + radius,
                      false, xmin, ymin);
            positions.push_back(Parameters::Vec3Type(nx, closest.y(), Parameters::zCenter));
        }
    }

    // find start / goal positions
    std::vector<float> distances;
    for (int i = 0; i < positions.size(); ++i)
    {
        for (int j = i+1; j < positions.size(); ++j)
        {
            distances.push_back((float)positions[i].distance(positions[j]));
        }
    }
    auto sorted = sort_indexes(distances);
    int difficultyIndex = std::max(0, std::min((int)distances.size()-1, (int)(difficulty * distances.size())));
    int count = 0;
    for (int i = 0; i < positions.size(); ++i)
    {
        for (int j = i+1; j < positions.size(); ++j)
        {
            if (count++ == difficultyIndex)
            {

                map._start = positions[i];
                map._goal = positions[j];
                break;
            }
        }
    }

    return map;
}

TrueMap TrueMap::operator=(TrueMap &map)
{
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
                auto v = map.search(x, y, z);
                this->updateNode(point, v->getOccupancy() > 0.5);
            }
        }
    }
    return *this;
}

TrueMap TrueMap::operator=(TrueMap map)
{
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
                auto v = map.search(x, y, z);
                this->updateNode(point, v == NULL ? true : v->getOccupancy() > 0.5);
            }
        }
    }
    return *this;
}

void TrueMap::shuffleCorridor(int radius, int branches, float difficulty, unsigned int seed)
{
    srand(seed);

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
                this->updateNode(point, true);
            }
        }
    }

    fillBlock(this, Parameters::voxelsPerDimensionX/2 - radius, Parameters::voxelsPerDimensionX/2 + radius,
              Parameters::voxelsPerDimensionY/2 - radius, Parameters::voxelsPerDimensionY/2 + radius);

    std::vector<Parameters::Vec3Type> positions({Parameters::Vec3Type(Parameters::xCenter, Parameters::yCenter, Parameters::zCenter)});
    for (int k = 0; k < branches; ++k)
    {
        int i = (int)(rand() * 1. / RAND_MAX * Parameters::voxelsPerDimensionX);
        int j = (int)(rand() * 1. / RAND_MAX * Parameters::voxelsPerDimensionY);
        // roll the dice again if we are too close to the center (double the chances of have more expanding maps)
//        while (std::abs(i - Parameters::voxelsPerDimensionX/2) < Parameters::voxelsPerDimensionX * 0.3
//                && std::abs(j - Parameters::voxelsPerDimensionY/2) < Parameters::voxelsPerDimensionY * 0.3)
//        {
//            i = (int)(rand() * 1. / RAND_MAX * Parameters::voxelsPerDimensionX);
//            j = (int)(rand() * 1. / RAND_MAX * Parameters::voxelsPerDimensionY);
//        }
        float nx = Parameters::xMin + i * Parameters::voxelSize;
        float ny = Parameters::yMin + j * Parameters::voxelSize;

        // find closest vertex
        double minDistance = std::numeric_limits<double>::max();
        Parameters::Vec3Type closest;
        for (auto &pos : positions)
        {
            double d = pos.distance(Parameters::Vec3Type(nx, ny, Parameters::zCenter));
            if (d < minDistance)
            {
                minDistance = d;
                closest = pos;
            }
        }

        int mi = (int) ((closest.x() - Parameters::xMin) / Parameters::voxelSize);
        int mj = (int) ((closest.y() - Parameters::yMin) / Parameters::voxelSize);

        if (std::abs(nx - closest.x()) < std::abs(ny - closest.y()))
        {
            // connect vertically
            fillBlock(this, mi - radius, mi + radius,
                      std::min(mj, j) - radius,
                      std::max(mj, j) + radius, false);
            positions.push_back(Parameters::Vec3Type(closest.x(), ny, Parameters::zCenter));
        }
        else
        {
            // connect horizontally
            fillBlock(this, std::min(mi, i) - radius, std::max(mi, i) + radius,
                      mj - radius, mj + radius, false);
            positions.push_back(Parameters::Vec3Type(nx, closest.y(), Parameters::zCenter));
        }
    }

    // find start / goal positions
    std::vector<float> distances;
    for (int i = 0; i < positions.size(); ++i)
    {
        for (int j = i+1; j < positions.size(); ++j)
        {
            distances.push_back((float)positions[i].distance(positions[j]));
        }
    }
    auto sorted = sort_indexes(distances);
    int difficultyIndex = std::max(0, std::min((int)distances.size()-1, (int)(difficulty * distances.size())));
    int count = 0;
    for (int i = 0; i < positions.size(); ++i)
    {
        for (int j = i+1; j < positions.size(); ++j)
        {
            if (count++ == difficultyIndex)
            {

                _start = positions[i];
                _goal = positions[j];
                break;
            }
        }
    }
}

Parameters::Vec3Type TrueMap::start() const
{
    return _start;
}

Parameters::Vec3Type TrueMap::goal() const
{
    return _goal;
}
