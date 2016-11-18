#include "../include/LogOddsMap.h"
#include "../include/Sensor.h"
#include "../include/Parameters.hpp"

#include <set>

LogOddsMap::LogOddsMap() : octomap::OcTree(Parameters::voxelSize), QVoxelMap(this)
{
}

struct OctoMapKeyComparator
{
    const bool operator()(const octomap::OcTreeKey &lhs, const octomap::OcTreeKey &rhs) const
    {
        return lhs[0] < rhs[0] && lhs[1] < rhs[3] && lhs[2] < rhs[2];
    }
};

bool LogOddsMap::update(const Observation &observation)
{
    lastUpdatedVoxels.clear();
    std::set<octomap::OcTreeKey, OctoMapKeyComparator> updatedKeys;
    for (auto &measurement : observation.measurements())
    {
        auto *ism = _computeInverseSensorModel(measurement);
        if (!ism)
        {
            ROS_WARN("Could not compute InverseSensorModel.");
            continue;
        }

        for (unsigned int i = 0; i < ism->ray.size(); ++i)
        {
            // update log odds
            updateNode(ism->ray[i], (float) ism->causeProbabilitiesOnRay[i]);
            updatedKeys.insert(ism->ray[i]);
            lastUpdatedVoxels.push_back(query(ism->ray[i]));
        }

        delete ism;
    }

    updateVisualization();

    return true;
}

LogOddsMap::InverseSensorModel *LogOddsMap::_computeInverseSensorModel(const Measurement &measurement) const
{
    octomap::KeyRay ray;
    if (!computeRayKeys(measurement.sensor->position(),
                        measurement.sensor->orientation() * measurement.sensor->range() + measurement.sensor->position(),
                        ray))
    {
        ROS_WARN("Compute ray keys failed.");
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
            auto distance = causeVoxel.position.distance(measurement.sensor->position());
            if (distance < measurement.value - Parameters::invSensor_rampSize / 2.)
            {
                ism->causeProbabilitiesOnRay[i] = Parameters::invSensor_free;
            }
            else if (distance < measurement.value + Parameters::invSensor_rampSize / 2.)
            {
                auto point_x = measurement.value - Parameters::invSensor_rampSize / 2.;
                auto point_y = Parameters::invSensor_free;
                ism->causeProbabilitiesOnRay[i] = point_y + Parameters::invSensor_rampSlope * (distance - point_x);
            }
            else if (distance < measurement.value + Parameters::invSensor_rampSize / 2. + Parameters::invSensor_topSize)
            {
                ism->causeProbabilitiesOnRay[i] = Parameters::invSensor_occupied;
            }
            else
            {
                ism->causeProbabilitiesOnRay[i] = 0;//Parameters::invSensor_prior;
            }

            assert(ism->causeProbabilitiesOnRay[i] < 1.);
            ++i;
        }

    }
    else if (measurement.geometry == GEOMETRY_HOLE)
    {
        ism->causeProbabilitiesOnRay = -0.05;//Parameters::invSensor_free;
    }
    else
    {
        ROS_WARN("Invalid measurement for InverseSensorModel computation.");
        return NULL;
    }

    return ism;
}