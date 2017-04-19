#pragma once

#include <unordered_set>
#include <octomap/OcTreeBaseImpl.h>
#include <ros/console.h>

#include "Parameters.hpp"
#include "BeliefVoxel.h"
#include "Observation.hpp"


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
    QVoxel(octomap::AbstractOcTreeNode *node, const octomap::point3d &position, GeometryType type, const octomap::OcTreeKey &key)
            : _node(node), position(position), type(type), key(key), hash(_hasher(key))
    {}
    QVoxel(octomap::AbstractOcTreeNode *node, const octomap::point3d &position, GeometryType type)
            : _node(node), position(position), type(type), hash(0)
    {}

    const octomap::AbstractOcTreeNode *_node;

private:
    __attribute__((weak))
    static const octomap::OcTreeKey::KeyHash _hasher;
};

const octomap::OcTreeKey::KeyHash QVoxel::_hasher = octomap::OcTreeKey::KeyHash();

template<class NODE>
class QTypedVoxel : public QVoxel
{
protected:
    QTypedVoxel(NODE *node, const octomap::point3d &position, GeometryType type, const octomap::OcTreeKey &key)
            : QVoxel(node, position, type, key)
    {}
    QTypedVoxel(NODE *node, const octomap::point3d &position, GeometryType type)
            : QVoxel(node, position, type)
    {}

public:
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

    NODE *node() const
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
            //ROS_WARN_STREAM("Voxel could not be found at position " << position);
            return QTypedVoxel<NODE>::hole(position);
        }
        octomap::OcTreeKey key = _tree->coordToKey(position);

        return QTypedVoxel<NODE>::voxel(node, position, key);
    }

    QTypedVoxel<NODE> query(const octomap::OcTreeKey &key) const
    {
        NODE *node = _tree->search(key);
        octomap::point3d position = _tree->keyToCoord(key);
        if (!node)
        {
            //ROS_WARN("Voxel could not be found at given key");
            return QTypedVoxel<NODE>::hole(position);
        }

        return QTypedVoxel<NODE>::voxel(node, position, key);
    }

    std::vector<QTypedVoxel<NODE> > voxels() const
    {
        std::vector<QTypedVoxel<NODE> > vs;
        for (unsigned int x = 0; x < Parameters::voxelsPerDimensionX; ++x)
        {
            for (unsigned int y = 0; y < Parameters::voxelsPerDimensionY; ++y)
            {
                for (unsigned int z = 0; z < Parameters::voxelsPerDimensionZ; ++z)
                {
                    octomap::point3d point(Parameters::xMin + x * Parameters::voxelSize,
                                           Parameters::yMin + y * Parameters::voxelSize,
                                           Parameters::zMin + z * Parameters::voxelSize);
                    vs.push_back(query(point));
                }
            }
        }
        return vs;
    }

    std::vector<QTypedVoxel<NODE> > voxels(const Parameters::KeySet &keys) const
    {
        std::vector<QTypedVoxel<NODE> > vs;
        for (auto &key: keys)
        {
            vs.push_back(query(key));
        }
        return vs;
    }

    /**
     * Linearly interpolates between 2 neighboring voxels per dimension around current point.
     * @param x X-coordinate of point.
     * @param y Y-coordinate of point.
     * @param z Z-coordinate of point.
     * @return Reachability 1 - m(voxel).
     */
    double filteredReachability(Parameters::NumType x, Parameters::NumType y, Parameters::NumType z)
    {
//        ROS_INFO("Filtering reachability %f %f %f", x, y, z);
        if (x <= Parameters::xMin || x >= Parameters::xMax ||
            y <= Parameters::yMin || y >= Parameters::yMax ||
            z < Parameters::zMin || z > Parameters::zMax)
        {
            return 1. - Parameters::priorMean; // TODO return 0 ?
        }
        int i = (int)std::floor((x - Parameters::xMin) / Parameters::voxelSize);
        int j = (int)std::floor((y - Parameters::yMin) / Parameters::voxelSize);
        int k = (int)std::floor((z - Parameters::zMin) / Parameters::voxelSize);
        double a = (x - Parameters::xMin) / Parameters::voxelSize - (double)i;
        double b = (y - Parameters::yMin) / Parameters::voxelSize - (double)j;
        double c = (z - Parameters::zMin) / Parameters::voxelSize - (double)k;
        int ip = std::min(i+1, (int)Parameters::voxelsPerDimensionX-1);
        int jp = std::min(j+1, (int)Parameters::voxelsPerDimensionY-1);
        int kp = std::min(k+1, (int)Parameters::voxelsPerDimensionZ-1);
        auto q000 = this->query(i * Parameters::voxelSize + Parameters::xMin,
                                j * Parameters::voxelSize + Parameters::yMin,
                                k * Parameters::voxelSize + Parameters::zMin);
        double m000 = getVoxelMean(q000);
        auto q100 = this->query(ip * Parameters::voxelSize + Parameters::xMin,
                                j * Parameters::voxelSize + Parameters::yMin,
                                k * Parameters::voxelSize + Parameters::zMin);
        double m100 = getVoxelMean(q100);
        auto q010 = this->query(i * Parameters::voxelSize + Parameters::xMin,
                                jp * Parameters::voxelSize + Parameters::yMin,
                                k * Parameters::voxelSize + Parameters::zMin);
        double m010 = getVoxelMean(q010);
        auto q001 = this->query(i * Parameters::voxelSize + Parameters::xMin,
                                j * Parameters::voxelSize + Parameters::yMin,
                                kp * Parameters::voxelSize + Parameters::zMin);
        double m001 = getVoxelMean(q001);
        auto q110 = this->query(ip * Parameters::voxelSize + Parameters::xMin,
                                jp * Parameters::voxelSize + Parameters::yMin,
                                k * Parameters::voxelSize + Parameters::zMin);
        double m110 = getVoxelMean(q110);
        auto q011 = this->query(i * Parameters::voxelSize + Parameters::xMin,
                                jp * Parameters::voxelSize + Parameters::yMin,
                                kp * Parameters::voxelSize + Parameters::zMin);
        double m011 = getVoxelMean(q011);
        auto q101 = this->query(ip * Parameters::voxelSize + Parameters::xMin,
                                j * Parameters::voxelSize + Parameters::yMin,
                                kp * Parameters::voxelSize + Parameters::zMin);
        double m101 = getVoxelMean(q101);
        auto q111 = this->query(ip * Parameters::voxelSize + Parameters::xMin,
                                jp * Parameters::voxelSize + Parameters::yMin,
                                kp * Parameters::voxelSize + Parameters::zMin);
        double m111 = getVoxelMean(q111);

        return std::pow(std::pow(std::pow(1.-m000, 1.-a) * std::pow(1.-m100, a), 1.-b)
                        * std::pow(std::pow(1.-m010, 1.-a) * std::pow(1.-m110, a), b), 1.-c)
                * std::pow(std::pow(std::pow(1.-m001, 1.-a) * std::pow(1.-m101, a), 1.-b)
                           * std::pow(std::pow(1.-m011, 1.-a) * std::pow(1.-m111, a), b), c);
    }

    /**
     * Linearly interpolates between 2 neighboring voxels per dimension around current point.
     * @param x X-coordinate of point.
     * @param y Y-coordinate of point.
     * @param z Z-coordinate of point.
     * @return Variance(voxel).
     */
    double filteredVariance(Parameters::NumType x, Parameters::NumType y, Parameters::NumType z)
    {
//        ROS_INFO("Filtering reachability %f %f %f", x, y, z);
        if (x < Parameters::xMin || x > Parameters::xMax ||
            y < Parameters::yMin || y > Parameters::yMax ||
            z < Parameters::zMin || z > Parameters::zMax)
        {
//            ROS_WARN("VARIANCE OUT OF BOUNDS!");
            return 1000;
        }
        int i = (int)std::floor((x - Parameters::xMin) / Parameters::voxelSize);
        int j = (int)std::floor((y - Parameters::yMin) / Parameters::voxelSize);
        int k = (int)std::floor((z - Parameters::zMin) / Parameters::voxelSize);
        double a = (x - Parameters::xMin) / Parameters::voxelSize - (double)i;
        double b = (y - Parameters::yMin) / Parameters::voxelSize - (double)j;
        double c = (z - Parameters::zMin) / Parameters::voxelSize - (double)k;
        int ip = std::min(i+1, (int)Parameters::voxelsPerDimensionX-1);
        int jp = std::min(j+1, (int)Parameters::voxelsPerDimensionY-1);
        int kp = std::min(k+1, (int)Parameters::voxelsPerDimensionZ-1);
        auto q000 = this->query(i * Parameters::voxelSize + Parameters::xMin,
                                j * Parameters::voxelSize + Parameters::yMin,
                                k * Parameters::voxelSize + Parameters::zMin);
        double m000 = std::pow(getVoxelStd(q000), 2.);
        auto q100 = this->query(ip * Parameters::voxelSize + Parameters::xMin,
                                j * Parameters::voxelSize + Parameters::yMin,
                                k * Parameters::voxelSize + Parameters::zMin);
        double m100 = std::pow(getVoxelStd(q100), 2.);
        auto q010 = this->query(i * Parameters::voxelSize + Parameters::xMin,
                                jp * Parameters::voxelSize + Parameters::yMin,
                                k * Parameters::voxelSize + Parameters::zMin);
        double m010 = std::pow(getVoxelStd(q010), 2.);
        auto q001 = this->query(i * Parameters::voxelSize + Parameters::xMin,
                                j * Parameters::voxelSize + Parameters::yMin,
                                kp * Parameters::voxelSize + Parameters::zMin);
        double m001 = std::pow(getVoxelStd(q001), 2.);
        auto q110 = this->query(ip * Parameters::voxelSize + Parameters::xMin,
                                jp * Parameters::voxelSize + Parameters::yMin,
                                k * Parameters::voxelSize + Parameters::zMin);
        double m110 = std::pow(getVoxelStd(q110), 2.);
        auto q011 = this->query(i * Parameters::voxelSize + Parameters::xMin,
                                jp * Parameters::voxelSize + Parameters::yMin,
                                kp * Parameters::voxelSize + Parameters::zMin);
        double m011 = std::pow(getVoxelStd(q011), 2.);
        auto q101 = this->query(ip * Parameters::voxelSize + Parameters::xMin,
                                j * Parameters::voxelSize + Parameters::yMin,
                                kp * Parameters::voxelSize + Parameters::zMin);
        double m101 = std::pow(getVoxelStd(q101), 2.);
        auto q111 = this->query(ip * Parameters::voxelSize + Parameters::xMin,
                                jp * Parameters::voxelSize + Parameters::yMin,
                                kp * Parameters::voxelSize + Parameters::zMin);
        double m111 = std::pow(getVoxelStd(q111), 2.);

        return std::pow(std::pow(std::pow(m000, 1.-a) * std::pow(m100, a), 1.-b)
                        * std::pow(std::pow(m010, 1.-a) * std::pow(m110, a), b), 1.-c)
               * std::pow(std::pow(std::pow(m001, 1.-a) * std::pow(m101, a), 1.-b)
                          * std::pow(std::pow(m011, 1.-a) * std::pow(m111, a), b), c);
    }

    virtual double getVoxelMean(QTypedVoxel<NODE> &voxel) const = 0;
    virtual double getVoxelStd(QTypedVoxel<NODE> &voxel) const = 0;

protected:
    QVoxelMap(const octomap::OcTreeBaseImpl<NODE, I> *tree) : _tree(tree)
    {}

private:
    const octomap::OcTreeBaseImpl<NODE, I> *_tree;
};