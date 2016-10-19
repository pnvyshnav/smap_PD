#include <stdlib.h>
#include <array>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <ros/console.h>

#include "../include/TrueMap.h"

TrueMap::TrueMap() : octomap::OcTree(Parameters::voxelSize), QVoxelMap(this)
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
