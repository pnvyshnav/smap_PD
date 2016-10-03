#pragma once

#include <array>
#include <algorithm>
#include <cmath>

#include <octomap/OcTreeDataNode.h>
#include <octomap/OcTreeBase.h>

#include "parameters.hpp"
#include "BeliefVoxel.h"

class BeliefMap : public octomap::OcTreeBaseImpl<BeliefVoxel, octomap::AbstractOcTree>
{
public:
    BeliefMap();

    std::string getTreeType() const;
    BeliefMap* create() const;

private:
};

