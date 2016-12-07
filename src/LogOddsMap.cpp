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
        return lhs[0] < rhs[0] && lhs[1] < rhs[1] && lhs[2] < rhs[2];
    }
};

bool LogOddsMap::update(const Observation &observation, const TrueMap &trueMap)
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

        bool obstacleReached = false;
        for (unsigned int i = 0; i < ism->ray.size(); ++i)
        {
            // update log odds
            updateNode(ism->ray[i], (float) ism->causeProbabilitiesOnRay[i]);
            updatedKeys.insert(ism->ray[i]);
            if (!obstacleReached)
                lastUpdatedVoxels.push_back(query(ism->ray[i]));

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
                ism->causeProbabilitiesOnRay[i] = 0;
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

std::vector<double> LogOddsMap::error(const TrueMap &trueMap) const
{
    std::vector<double> err;
    for (unsigned int x = 0; x < Parameters::voxelsPerDimensionX; ++x)
    {
        for (unsigned int y = 0; y < Parameters::voxelsPerDimensionY; ++y)
        {
            for (unsigned int z = 0; z < Parameters::voxelsPerDimensionZ; ++z)
            {
                octomap::point3d point(Parameters::xMin + x * Parameters::voxelSize,
                                       Parameters::yMin + y * Parameters::voxelSize,
                                       Parameters::zMin + z * Parameters::voxelSize);
                auto trueVoxel = trueMap.query(point);
                auto estimated = query(point);
                if (estimated.type == GEOMETRY_VOXEL)
                    err.push_back(std::round(trueVoxel.node()->getOccupancy())-estimated.node()->getOccupancy());
                else
                    // TODO consider holes as voxels with 0.5 occupancy
                    err.push_back(std::round(trueVoxel.node()->getOccupancy()) - Parameters::priorMean);
            }
        }
    }
    return err;
}

std::vector<double> LogOddsMap::errorLastUpdated(const TrueMap &trueMap) const
{
    std::vector<double> err;
    for (auto &v : lastUpdatedVoxels)
    {
        auto trueVoxel = trueMap.query(v.position);
        auto estimated = query(v.position);
        if (!trueVoxel.node())
            continue;
        if (estimated.type == GEOMETRY_VOXEL)
            err.push_back(std::round(trueVoxel.node()->getOccupancy())-estimated.node()->getOccupancy());
        else
            // TODO consider holes as voxels with 0.5 occupancy
            err.push_back(std::round(trueVoxel.node()->getOccupancy()) - Parameters::priorMean);
    }
    return err;
}

void LogOddsMap::reset()
{
    for (unsigned int x = 0; x < Parameters::voxelsPerDimensionX; ++x)
    {
        for (unsigned int y = 0; y < Parameters::voxelsPerDimensionY; ++y)
        {
            for (unsigned int z = 0; z < Parameters::voxelsPerDimensionZ; ++z)
            {
                octomap::point3d point((float) (Parameters::xMin + x * Parameters::voxelSize),
                                       (float) (Parameters::yMin + y * Parameters::voxelSize),
                                       (float) (Parameters::zMin + z * Parameters::voxelSize));
                auto voxel = search(point);
                if (voxel == NULL)
                    continue;
                voxel->setValue((float) Parameters::priorMean);
            }
        }
    }
    publish();
}
