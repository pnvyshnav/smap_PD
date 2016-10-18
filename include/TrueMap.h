#pragma once

#include <octomap/OcTree.h>
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
struct QVoxel
{
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

    static QVoxel voxel(Parameters::NumType occupancy, octomap::point3d position, octomap::OcTreeKey key)
    {
        return QVoxel(occupancy, position, GEOMETRY_VOXEL, key);
    }

private:
	QVoxel(Parameters::NumType occupancy, octomap::point3d position, GeometryType type, octomap::OcTreeKey key = octomap::OcTreeKey())
			: occupancy(occupancy), position(position), type(type), key(key)
	{}
};

class TrueMap : public octomap::OcTree
{
public:
	static TrueMap generate(unsigned int seed = 1);

	QVoxel query(octomap::point3d &position) const;
    QVoxel query(octomap::OcTreeKey key) const;

private:
    TrueMap();
};

