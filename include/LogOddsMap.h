#pragma once

#include <valarray>
#include <octomap/OcTree.h>
#include "Parameters.hpp"
#include "QVoxel.hpp"
#include "Observable.hpp"
#include "Observation.hpp"

class LogOddsMap
        : public octomap::OcTree,
          public QVoxelMap<octomap::OcTreeNode, octomap::AbstractOccupancyOcTree>,
          public Observable
{
public:
    LogOddsMap();

    std::vector<QTrueVoxel> lastUpdatedVoxels;

    // TODO remove trueMap argument
    bool update(const Observation &observation, const TrueMap &trueMap);

    std::vector<double> error(const TrueMap &trueMap) const;
    std::vector<double> errorLastUpdated(const TrueMap &trueMap) const;

    void reset();

private:
    struct InverseSensorModel
    {
        std::valarray<Parameters::NumType> causeProbabilitiesOnRay;
        std::vector<octomap::OcTreeKey> ray;
    };

    InverseSensorModel *_computeInverseSensorModel(const Measurement &measurement);
};