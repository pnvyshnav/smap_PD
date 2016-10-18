#include "../include/TrueMap.h"

#include <stdlib.h> 
#include <array>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <ros/console.h>

#include "../include/Parameters.hpp"

TrueMap::TrueMap() : octomap::OcTree(Parameters::voxelSize)
{
}

TrueMap TrueMap::generate(unsigned int seed)
{
    TrueMap map;
    srand(seed);
    octomap::point3d center(Parameters::xCenter, Parameters::yCenter, Parameters::zCenter);
    for (Parameters::NumType x = Parameters::xMin; x <= Parameters::xMax; x += Parameters::voxelSize)
    {
        for (Parameters::NumType y = Parameters::yMin; y <= Parameters::yMax; y += Parameters::voxelSize)
        {
            for (Parameters::NumType z = Parameters::zMin; z <= Parameters::zMax; z += Parameters::voxelSize)
            {
                octomap::point3d point(x, y, z);
                // cells around center (where the robot is) are free
                if (center.distance(point) <= Parameters::freeRadiusAroundCenter)
                {
                    map.updateNode(point, false);
                }
                else
                {
                    bool occupied = rand() % 3 == 1; // 1/3 occupied
                    map.updateNode(point, occupied);
                }
            }
        }
    }
    return map;
}

TrueVoxel TrueMap::query(octomap::point3d &position) const
{
    octomap::OcTreeNode *node = search(position);
    if (!node)
    {
        ROS_WARN_STREAM("Voxel could not be found at position" << position);
        return TrueVoxel(-1., position, false);
    }
    return TrueVoxel(node->getOccupancy(), position, true);
}
