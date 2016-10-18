#pragma once

#include <octomap/OcTree.h>

struct TrueVoxel
{
	const double occupancy;
	const octomap::point3d position;
	const bool existent;

	TrueVoxel(double occupancy, octomap::point3d position, bool existent)
			: occupancy(occupancy), position(position), existent(existent)
	{}
};

class TrueMap : public octomap::OcTree
{
public:
	static TrueMap generate(unsigned int seed = 1);

	TrueVoxel query(octomap::point3d &position) const;

private:
    TrueMap();
};

