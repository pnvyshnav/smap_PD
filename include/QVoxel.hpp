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
class QVoxel
{
public:
    const octomap::point3d position;
    const GeometryType type;
    const octomap::OcTreeKey key;
    const size_t hash;

protected:
    QVoxel(void *node, const octomap::point3d &position, GeometryType type, const octomap::OcTreeKey &key = octomap::OcTreeKey())
            : _node(node), position(position), type(type), key(key), hash(_hasher(key))
    {}

    const void *_node;

private:
    __attribute__((weak))
    static const octomap::OcTreeKey::KeyHash _hasher;
};

const octomap::OcTreeKey::KeyHash QVoxel::_hasher = octomap::OcTreeKey::KeyHash();

template<class NODE>
class QTypedVoxel : public QVoxel
{
public:
    QTypedVoxel(NODE *node, const octomap::point3d &position, GeometryType type, const octomap::OcTreeKey &key = octomap::OcTreeKey())
            : QVoxel(node, position, type, key) {}

    static QTypedVoxel<NODE> hole(const octomap::point3d &position = octomap::point3d())
    {
        return QTypedVoxel<NODE>(NULL, position, GEOMETRY_HOLE);
    }

    static QTypedVoxel<NODE> spurious(const octomap::point3d &position = octomap::point3d())
    {
        return QTypedVoxel<NODE>(NULL, position, GEOMETRY_SPURIOUS);
    }

    static QTypedVoxel<NODE> voxel(NODE *node, const octomap::point3d &position, const octomap::OcTreeKey &key)
    {
        return QTypedVoxel<NODE>(node, position, GEOMETRY_VOXEL, key);
    }

    const NODE *node() const
    {
        return (NODE*)_node;
    }
};

typedef QTypedVoxel<octomap::OcTreeNode> QTrueVoxel;
typedef QTypedVoxel<BeliefVoxel> QBeliefVoxel;

/**
 * Mixin to provide support for QVoxel querying.
 */
template<class NODE, class I>
class QVoxelMap
{
public:
    QTypedVoxel<NODE> query(Parameters::NumType x, Parameters::NumType y, Parameters::NumType z) const
    {
        auto p = octomap::point3d(x, y, z);
        return query(p);
    }

    QTypedVoxel<NODE> query(const octomap::point3d &position) const
    {
        NODE *node = _tree->search(position);
        if (!node)
        {
            ROS_WARN_STREAM("Voxel could not be found at position " << position);
            return QTypedVoxel<NODE>::hole(position);
        }
        octomap::OcTreeKey key = _tree->coordToKey(position);

        return QTypedVoxel<NODE>::voxel(node, position, key);
    }

    QTypedVoxel<NODE> query(octomap::OcTreeKey key) const
    {
        NODE *node = _tree->search(key);
        octomap::point3d position = _tree->keyToCoord(key);
        if (!node)
        {
            ROS_WARN("Voxel could not be found at given key");
            return QTypedVoxel<NODE>::hole(position);
        }

        return QTypedVoxel<NODE>::voxel(node, position, key);
    }

protected:
    QVoxelMap(const octomap::OcTreeBaseImpl<NODE, I> *tree) : _tree(tree)
    {}

private:
    const octomap::OcTreeBaseImpl<NODE, I> *_tree;
};