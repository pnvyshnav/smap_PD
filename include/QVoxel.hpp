#pragma once

#include <octomap/OcTreeBaseImpl.h>
#include <ros/console.h>
#include "Parameters.hpp"

enum GeometryType
{
    GEOMETRY_VOXEL,
    GEOMETRY_SPURIOUS,
    GEOMETRY_HOLE
};

/**
 * QVoxel stands for a "queried" voxel, i.e. it might be a hole or spurious geometry
 * if there is no occupancy information at the given position.
 */
class QVoxel
{
public:
    const Parameters::NumType occupancy;
    const octomap::point3d position;
    const GeometryType type;
    const octomap::OcTreeKey key;

    static QVoxel hole()
    {
        return QVoxel((Parameters::NumType) -1., octomap::point3d(), GEOMETRY_HOLE);
    }

    static QVoxel spurious()
    {
        return QVoxel((Parameters::NumType) -1., octomap::point3d(), GEOMETRY_SPURIOUS);
    }

    static QVoxel voxel(Parameters::NumType occupancy, const octomap::point3d &position, const octomap::OcTreeKey &key)
    {
        return QVoxel(occupancy, position, GEOMETRY_VOXEL, key);
    }

private:
    QVoxel(Parameters::NumType occupancy, const octomap::point3d &position, GeometryType type, const octomap::OcTreeKey &key = octomap::OcTreeKey())
            : occupancy(occupancy), position(position), type(type), key(key)
    {}
};

/**
 * Mixin to provide inherited classes support for QVoxel querying.
 */
template<class NODE, class I>
class QVoxelMap
{
public:
    QVoxel query(octomap::point3d &position) const
    {
        octomap::OcTreeNode *node = _tree->search(position);
        if (!node)
        {
            ROS_WARN_STREAM("Voxel could not be found at position" << position);
            return QVoxel::hole();
        }
        octomap::OcTreeKey key = _tree->coordToKey(position);
        return QVoxel::voxel((Parameters::NumType) node->getOccupancy(), position, key);
    }

    QVoxel query(octomap::OcTreeKey key) const
    {
        octomap::OcTreeNode *node = _tree->search(key);
        octomap::point3d position = _tree->keyToCoord(key);
        if (!node)
        {
            ROS_WARN_STREAM("Voxel could not be found at position" << position);
            return QVoxel::hole();
        }
        return QVoxel::voxel((Parameters::NumType) node->getOccupancy(), position, key);
    }

protected:
    QVoxelMap(octomap::OcTreeBaseImpl<NODE, I> *tree) : _tree(tree)
    {}

private:
    const octomap::OcTreeBaseImpl<NODE, I> *_tree;
};