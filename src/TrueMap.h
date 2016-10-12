#pragma once

#include <octomap/OcTree.h>

class TrueMap : public octomap::OcTree
{
public:
	static TrueMap generate(unsigned int seed = 1);

private:
    TrueMap();
};

