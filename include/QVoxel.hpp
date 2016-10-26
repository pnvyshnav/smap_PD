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
    const size_t hash;

    static QVoxel hole(const octomap::point3d &position = octomap::point3d())
    {
        return QVoxel(NULL, position, GEOMETRY_HOLE);
    }

    static QVoxel spurious(const octomap::point3d &position = octomap::point3d())
    {
        return QVoxel(NULL, position, GEOMETRY_SPURIOUS);
    }

    static QVoxel voxel(NODE *node, const octomap::point3d &position, const octomap::OcTreeKey &key)
    {
        return QVoxel(node, position, GEOMETRY_VOXEL, key);
    }

private:
    QVoxel(NODE *node, const octomap::point3d &position, GeometryType type, const octomap::OcTreeKey &key = octomap::OcTreeKey())
            : node(node), position(position), type(type), key(key), hash(_hasher(key))
    {}

    static const octomap::OcTreeKey::KeyHash _hasher;
};

template<class NODE>
const octomap::OcTreeKey::KeyHash QVoxel<NODE>::_hasher = octomap::OcTreeKey::KeyHash();

typedef QVoxel<octomap::OcTreeNode> QTrueVoxel;
typedef QVoxel<BeliefVoxel> QBeliefVoxel;

/**
 * Mixin to provide support for QVoxel querying.
 */
template<class NODE, class I>
class QVoxelMap
{
public:
    QVoxel<NODE> query(Parameters::NumType x, Parameters::NumType y, Parameters::NumType z) const
    {
        auto p = octomap::point3d(x, y, z);
        return query(p);
    }

    QVoxel<NODE> query(const octomap::point3d &position) const
    {
        NODE *node = _tree->search(position);
        if (!node)
        {
            ROS_WARN_STREAM("Voxel could not be found at position " << position);
            return QVoxel<NODE>::hole(position);
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
            ROS_WARN("Voxel could not be found at given key");
            return QVoxel<NODE>::hole(position);
        }

        return QVoxel<NODE>::voxel(node, position, key);
    }

protected:
    QVoxelMap(const octomap::OcTreeBaseImpl<NODE, I> *tree) : _tree(tree)
    {}

private:
    const octomap::OcTreeBaseImpl<NODE, I> *_tree;
};