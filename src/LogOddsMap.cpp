#include "../include/LogOddsMap.h"
#include "../include/Sensor.h"
#include "../include/Parameters.hpp"

#include <set>

ISMParameters LogOddsMap::parameters;

LogOddsMap::LogOddsMap() : octomap::OcTree(Parameters::voxelSize), StatisticsMap(this)
{
    this->setClampingThresMin(0.);
    this->setClampingThresMax(1.);
}

struct OctoMapKeyComparator
{
    const bool operator()(const octomap::OcTreeKey &lhs, const octomap::OcTreeKey &rhs) const
    {
        return lhs[0] < rhs[0] && lhs[1] < rhs[1] && lhs[2] < rhs[2];
    }
};

bool LogOddsMap::update(const Observation &observation, const TrueMap &trueMap)
{
    _lastUpdatedVoxels.clear();
    std::set<octomap::OcTreeKey, OctoMapKeyComparator> updatedKeys;
    for (auto &measurement : observation.measurements())
    {
        auto *ism = _computeInverseSensorModel(measurement);
        if (!ism)
        {
            ROS_WARN("Could not compute InverseSensorModel.");
            continue;
        }

        bool obstacleReached = false;
        for (unsigned int i = 0; i < ism->ray.size(); ++i)
        {
            // update log odds
            updateNode(ism->ray[i], (float) ism->causeProbabilitiesOnRay[i]);
            updatedKeys.insert(ism->ray[i]);
            if (!obstacleReached)
                _lastUpdatedVoxels.push_back(query(ism->ray[i]));

            // TODO for evaluation purposes, only take voxels that lie in front of an obstacle
            auto trueVoxel = trueMap.query(ism->ray[i]);
            if (trueVoxel.type == GEOMETRY_HOLE || ((int)std::round(trueVoxel.node()->getOccupancy())) == 1)
                obstacleReached = true;
        }

        delete ism;
    }

    updateSubscribers();

    return true;
}

LogOddsMap::InverseSensorModel *LogOddsMap::_computeInverseSensorModel(const Measurement &measurement)
{
    octomap::KeyRay ray;
    if (!computeRayKeys(measurement.sensor.position,
                        measurement.sensor.orientation * measurement.sensor.range + measurement.sensor.position,
                        ray))
    {
        ROS_WARN("Compute _ray keys failed.");
        return NULL;
    }

    if (ray.size() == 0)
    {
        return NULL;
    }

    InverseSensorModel *ism = new InverseSensorModel;
    unsigned int j = 0;
    std::vector<QVoxel> causeVoxels;
    for (auto &key : ray)
    {
        auto causeVoxel = query(key);
        if (!TrueMap::insideMap(causeVoxel.position))
            continue;
        ism->ray.push_back(key);
        causeVoxels.push_back(causeVoxel);
        ++j;
        if (j >= ray.size())
            break;
    }

    ism->causeProbabilitiesOnRay = std::valarray<Parameters::NumType>(ism->ray.size());

    if (measurement.geometry == GEOMETRY_VOXEL)
    {
        unsigned int i = 0;
        for (auto &causeVoxel : causeVoxels)
        {
            auto distance = causeVoxel.position.distance(measurement.sensor.position);
            if (distance < measurement.value - parameters.rampSize / 2.)
            {
                ism->causeProbabilitiesOnRay[i] = parameters.free;
            }
            else if (distance < measurement.value + parameters.rampSize / 2.)
            {
                auto point_x = measurement.value - parameters.rampSize / 2.;
                auto point_y = parameters.free;
                ism->causeProbabilitiesOnRay[i] = point_y + parameters.rampSlope * (distance - point_x);
            }
            else if (distance < measurement.value + parameters.rampSize / 2. + parameters.topSize)
            {
                ism->causeProbabilitiesOnRay[i] = parameters.occupied;
            }
            else
            {
                ism->causeProbabilitiesOnRay[i] = 0;
            }

            assert(ism->causeProbabilitiesOnRay[i] < 1.);
            ++i;
        }

    }
    else if (measurement.geometry == GEOMETRY_HOLE)
    {
        ism->causeProbabilitiesOnRay = parameters.free;
    }
    else
    {
        ROS_WARN("Invalid measurement for InverseSensorModel computation.");
        return NULL;
    }

    return ism;
}

void LogOddsMap::reset()
{
    this->clear();
    _lastUpdatedVoxels.clear();
    publish();
}