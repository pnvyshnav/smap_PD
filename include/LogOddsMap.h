#pragma once

#include <valarray>
#include <octomap/OcTree.h>
#include "Parameters.hpp"
#include "StatisticsMap.hpp"
#include "Observable.hpp"
#include "Observation.hpp"
#include "TrueMap.h"

struct ISMParameters
{
    double increment = Parameters::invSensor_increment;
    double rampSize = Parameters::invSensor_rampSize;
    double topSize = Parameters::invSensor_topSize;
    double free = -increment;
    double occupied = increment;
    double rampSlope = Parameters::invSensor_rampSlope;

    void setIncrement(double inc)
    {
        increment = inc;
        free = -increment;
        occupied = increment;
        rampSlope = (occupied - free)/rampSize;
    }

    void setRampSize(double size)
    {
        rampSize = size;
        rampSlope = (occupied - free)/rampSize;
    }

    void setTopSize(double size)
    {
        topSize = size;
    }

};

class LogOddsMap
        : public octomap::OcTree,
          public StatisticsMap<octomap::OcTreeNode, octomap::AbstractOccupancyOcTree>,
          public Observable
{
public:
    LogOddsMap();

    static ISMParameters parameters;

    // TODO remove trueMap argument
    bool update(const Observation &observation, const TrueMap &trueMap);

    void reset();

    std::vector<QTrueVoxel> updatedVoxels() const
    {
        return _lastUpdatedVoxels;
    }

    double getVoxelMean(QTrueVoxel &voxel) const
    {
        if (!voxel.node())
            return Parameters::priorMean;
        return voxel.node()->getOccupancy();
    }

    double getVoxelStd(QTrueVoxel &voxel) const
    {
        if (!voxel.node())
            return Parameters::priorStd;
        double p = getVoxelMean(voxel);
        return std::sqrt(p * (1. - p));
    }

    std::string mapType() const
    {
        return "LogOdds";
    }

private:
    struct InverseSensorModel
    {
        std::valarray<Parameters::NumType> causeProbabilitiesOnRay;
        std::vector<octomap::OcTreeKey> ray;
    };

    InverseSensorModel *_computeInverseSensorModel(const Measurement &measurement);
    std::vector<QTrueVoxel> _lastUpdatedVoxels;
};