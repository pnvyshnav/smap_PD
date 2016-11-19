#pragma once

#include <valarray>
#include <octomap/OcTree.h>
#include "Parameters.hpp"
#include "QVoxel.hpp"
#include "Visualizable.hpp"
#include "Observation.hpp"

class LogOddsMap
        : public octomap::OcTree,
          public QVoxelMap<octomap::OcTreeNode, octomap::AbstractOccupancyOcTree>,
          public Visualizable
{
public:
    LogOddsMap();

    std::vector<QTrueVoxel> lastUpdatedVoxels;

    bool update(const Observation &observation);

    double error(const TrueMap &trueMap) const;

private:
    struct InverseSensorModel
    {
        std::valarray<Parameters::NumType> causeProbabilitiesOnRay;
        std::vector<octomap::OcTreeKey> ray;
    };

    InverseSensorModel *_computeInverseSensorModel(const Measurement &measurement) const;
};