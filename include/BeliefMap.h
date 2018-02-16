#pragma once

#include <array>
#include <valarray>
#include <algorithm>
#include <cmath>

#include <octomap/OcTreeDataNode.h>
#include <octomap/OcTreeBase.h>

#include "Parameters.h"
#include "BeliefVoxel.h"
#include "StatisticsMap.hpp"
#include "Observation.hpp"
#include "Observable.hpp"
#include "TrueMap.h"
#include "InverseCauseModel.h"

class InverseCauseModel;
class BeliefMap
        : public octomap::OcTreeBaseImpl<BeliefVoxel, octomap::AbstractOcTree>,
          public StatisticsMap<BeliefVoxel, octomap::AbstractOcTree>,
          public Observable
{
public:
    BeliefMap();
    BeliefMap(const BeliefMap &map);

    BeliefMap *create() const;

    BeliefMap copy() const;

    BeliefMap &operator=(const BeliefMap &map);

    BeliefDistribution belief(const octomap::OcTreeKey &key) const;

    BeliefVoxel *updateNode(const octomap::point3d& position, const BeliefDistribution &belief);
    BeliefVoxel *updateNode(const octomap::OcTreeKey& key, const BeliefDistribution &belief);

    std::valarray<Parameters::NumType> bouncingProbabilitiesOnRay(
            const octomap::KeyRay &ray) const;

    std::valarray<Parameters::NumType> reachingProbabilitiesOnRay(
            const octomap::KeyRay &ray,
            const std::valarray<Parameters::NumType> &bouncingProbabilities) const;

    // TODO remove trueMap argument
    bool update(const Observation &observation, const TrueMap &trueMap);

    void reset();

    std::vector<QBeliefVoxel> updatedVoxels() const
    {
        return _lastUpdatedVoxels;
    }

    double getVoxelMean(QBeliefVoxel &voxel) const
    {
        if (voxel.type != GEOMETRY_VOXEL)
            return Parameters::priorMean;
        return voxel.node()->getValue().mean();
    }

    double getVoxelStd(QBeliefVoxel &voxel) const
    {
        if (voxel.type != GEOMETRY_VOXEL)
            return Parameters::priorStd;
        return std::sqrt(voxel.node()->getValue().variance()) * StdDevScalingFactor;
    }

    std::string mapType() const
    {
        return "Belief";
    }

    InverseCauseModel icm() const
    {
        return _icm;
    }

    std::vector<Parameters::NumType> particles() const;

    // TODO std scaling such that initial std equals defined prior std (0.5)
    static constexpr double StdDevScalingFactor = Parameters::priorStd / std::sqrt(0.085);

    friend std::ostream& operator<<(std::ostream &os, const BeliefMap &map)
    {
        double params[7] = { Parameters::voxelSize,
                             Parameters::xMin,
                             Parameters::xMax,
                             Parameters::yMin,
                             Parameters::yMax,
                             Parameters::zMin,
                             Parameters::zMax };
        os.write((char *) &params, sizeof params);
        os.write(reinterpret_cast<const char *>(&Parameters::numParticles), sizeof(Parameters::numParticles));

        for (unsigned int x = 0; x < Parameters::voxelsPerDimensionX(); ++x)
        {
            for (unsigned int y = 0; y < Parameters::voxelsPerDimensionY(); ++y)
            {
                for (unsigned int z = 0; z < Parameters::voxelsPerDimensionZ(); ++z)
                {
                    octomap::point3d point(Parameters::xMin + x * Parameters::voxelSize,
                                           Parameters::yMin + y * Parameters::voxelSize,
                                           Parameters::zMin + z * Parameters::voxelSize);
                    auto belief = map.search(point)->getValue();
                    os << belief;
                }
            }
        }

        return os;
    }

    friend std::istream& operator>>(std::istream &is, BeliefMap &map)
    {
        std::cout << Parameters::voxelSize << std::endl;
        std::cout << Parameters::xMin << std::endl;
        std::cout << Parameters::xMax << std::endl;
        std::cout << Parameters::yMin << std::endl;
        std::cout << Parameters::yMax << std::endl;
        std::cout << Parameters::zMin << std::endl;
        std::cout << Parameters::zMax << std::endl;
        std::cout << Parameters::numParticles << std::endl;

        std::cout << std::endl;

        is >> Parameters::voxelSize;
        is >> Parameters::xMin;
        is >> Parameters::xMax;
        is >> Parameters::yMin;
        is >> Parameters::yMax;
        is >> Parameters::zMin;
        is >> Parameters::zMax;
        is >> Parameters::numParticles;

        std::cout << Parameters::voxelSize << std::endl;
        std::cout << Parameters::xMin << std::endl;
        std::cout << Parameters::xMax << std::endl;
        std::cout << Parameters::yMin << std::endl;
        std::cout << Parameters::yMax << std::endl;
        std::cout << Parameters::zMin << std::endl;
        std::cout << Parameters::zMax << std::endl;
        std::cout << Parameters::numParticles << std::endl;

        map._initialize();

        for (unsigned int x = 0; x < Parameters::voxelsPerDimensionX(); ++x)
        {
            for (unsigned int y = 0; y < Parameters::voxelsPerDimensionY(); ++y)
            {
                for (unsigned int z = 0; z < Parameters::voxelsPerDimensionZ(); ++z)
                {
                    octomap::point3d point(Parameters::xMin + x * Parameters::voxelSize,
                                           Parameters::yMin + y * Parameters::voxelSize,
                                           Parameters::zMin + z * Parameters::voxelSize);
                    BeliefDistribution belief(false);
                    is >> belief;
                    map.updateNode(point, belief);
                }
            }
        }

//    ROS_INFO("BeliefDistribution map has %d nodes in total.", (int) calcNumNodes());
        map._calcMinMax();
        return is;
    }

    bool save(std::string filename) const
    {
        std::fstream file(filename, std::ios::out | std::ios::binary);
        file << *this;
        file.close();
        return !file.bad();
    }

    bool loadFromFile(std::string filename)
    {
        std::fstream file(filename, std::ios::in | std::ios::binary);
        file >> *this;
        file.close();
        return !file.bad();
    }

protected:
    BeliefVoxel *updateNodeRecurs(BeliefVoxel *node, bool node_just_created,
                                  const octomap::OcTreeKey &key,
                                  unsigned int depth, const BeliefDistribution &belief);
    void _initialize();
    void _calcMinMax();

private:
    std::vector<QBeliefVoxel> _lastUpdatedVoxels;

    InverseCauseModel _icm;

    void _expandNode(BeliefVoxel *node);
    BeliefVoxel *_createNodeChild(BeliefVoxel *node, unsigned int childIdx);
};
