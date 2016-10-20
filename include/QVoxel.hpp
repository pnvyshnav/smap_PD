#pragma once

#include <octomap/OcTreeBaseImpl.h>
#include <ros/console.h>
#include "Parameters.hpp"
#include "BeliefVoxel.h"

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
template<class NODE>
class QVoxel
{
public:
    const NODE *node;
    const octomap::point3d position;
    const GeometryType type;
    const octomap::OcTreeKey key;

    static QVoxel hole()
    {
        return QVoxel(NULL, octomap::point3d(), GEOMETRY_HOLE);
    }

    static QVoxel spurious()
    {
        return QVoxel(NULL, octomap::point3d(), GEOMETRY_SPURIOUS);
    }

    static QVoxel voxel(NODE *node, const octomap::point3d &position, const octomap::OcTreeKey &key)
    {
        return QVoxel(node, position, GEOMETRY_VOXEL, key);
    }

private:
    QVoxel(NODE *node, const octomap::point3d &position, GeometryType type, const octomap::OcTreeKey &key = octomap::OcTreeKey())
            : node(node), position(position), type(type), key(key)
    {}
};

typedef QVoxel<octomap::OcTreeNode> QTrueVoxel;
typedef QVoxel<BeliefVoxel> QBeliefVoxel;

/**
 * Mixin to provide inherited classes support for QVoxel querying.
 */
template<class NODE, class I>
class QVoxelMap
{
public:
    QVoxel<NODE> query(octomap::point3d &position) const
    {
        NODE *node = _tree->search(position);
        if (!node)
        {
            ROS_WARN_STREAM("Voxel could not be found at position" << position);
            return QVoxel<NODE>::hole();
        }
        octomap::OcTreeKey key = _tree->coordToKey(position);

        return QVoxel<NODE>::voxel(node, position, key);
    }

    QVoxel<NODE> query(octomap::OcTreeKey key) const
    {
        NODE *node = _tree->search(key);
        octomap::point3d position = _tree->keyToCoord(key);
        if (!node)
        {
            ROS_WARN_STREAM("Voxel could not be found at position" << position);
            return QVoxel<NODE>::hole();
        }

        return QVoxel<NODE>::voxel(node, position, key);
    }

protected:
    QVoxelMap(octomap::OcTreeBaseImpl<NODE, I> *tree) : _tree(tree)
    {}

private:
    const octomap::OcTreeBaseImpl<NODE, I> *_tree;
};