#pragma once

#include "QVoxel.hpp"
#include "TrueMap.h"

struct VoxelStatistics
{
    double mean;
    bool trueMean;
    double error;
    double stdDev;
    const QVoxel *voxel;
    VoxelStatistics(bool trueMean, double mean, double error, double stdDev, const QVoxel *voxel)
            : trueMean(trueMean), mean(mean), error(error), stdDev(stdDev), voxel(voxel)
    {}

    static std::vector<double> selectTrueMean(const std::vector<VoxelStatistics> &stats)
    {
        std::vector<double> vs(stats.size());
        unsigned int i = 0;
        for (auto &stat : stats)
            vs[i++] = (int)stat.trueMean;
        return vs;
    }

    static std::vector<double> selectMean(const std::vector<VoxelStatistics> &stats)
    {
        std::vector<double> vs(stats.size());
        unsigned int i = 0;
        for (auto &stat : stats)
            vs[i++] = stat.mean;
        return vs;
    }

    static std::vector<double> selectError(const std::vector<VoxelStatistics> &stats)
    {
        std::vector<double> vs(stats.size());
        unsigned int i = 0;
        for (auto &stat : stats)
            vs[i++] = stat.error;
        return vs;
    }

    static std::vector<double> selectStd(const std::vector<VoxelStatistics> &stats)
    {
        std::vector<double> vs(stats.size());
        unsigned int i = 0;
        for (auto &stat : stats)
            vs[i++] = stat.stdDev;
        return vs;
    }

    static std::vector<octomap::OcTreeKey> selectKey(const std::vector<VoxelStatistics> &stats)
    {
        std::vector<octomap::OcTreeKey> vs(stats.size());
        unsigned int i = 0;
        for (auto &stat : stats)
            vs[i++] = stat.voxel->key;
        return vs;
    }
};

template<class NODE, class I>
class StatisticsMap : public QVoxelMap<NODE, I>
{
public:
    std::vector<VoxelStatistics> stats(const TrueMap &trueMap) const
    {
        std::vector<VoxelStatistics> stats;
        for (auto &voxel : this->voxels())
        {
            QTrueVoxel trueVoxel = trueMap.query(voxel.key);
            double trueMean = trueMap.getVoxelMean(trueVoxel);
            if (voxel.type != GEOMETRY_VOXEL)
            {
//                ROS_WARN_STREAM("Skipped stats computation for a voxel in "
//                                        << mapType() << " at position: " << voxel.position);
                double bernoulliStd = std::sqrt(Parameters::priorMean * (1. - Parameters::priorMean));
                stats.push_back(VoxelStatistics(trueMean > 0.5, Parameters::priorMean,
                                                Parameters::priorMean, bernoulliStd, &voxel));
                continue;
            }
            double mean = this->getVoxelMean(voxel);
            double error = (trueMean - mean);
            stats.push_back(VoxelStatistics(trueMean > 0.5, mean, error, this->getVoxelStd(voxel), &voxel));
        }
        return stats;
    }

    std::vector<VoxelStatistics> stats(const TrueMap &trueMap, const Parameters::KeySet &keys) const
    {
        std::vector<VoxelStatistics> stats;
        for (auto &voxel : this->voxels(keys))
        {
            QTrueVoxel trueVoxel = trueMap.query(voxel.key);
            double trueMean = trueMap.getVoxelMean(trueVoxel);
            if (voxel.type != GEOMETRY_VOXEL)
            {
//                ROS_WARN_STREAM("Skipped stats computation for a voxel in "
//                                        << mapType() << " at position: " << voxel.position);
                double bernoulliStd = std::sqrt(Parameters::priorMean * (1. - Parameters::priorMean));
                stats.push_back(VoxelStatistics(trueMean > 0.5, Parameters::priorMean,
                                                Parameters::priorMean, bernoulliStd, &voxel));
                continue;
            }
            double mean = this->getVoxelMean(voxel);
            double error = (trueMean - mean);
            stats.push_back(VoxelStatistics(trueMean > 0.5, mean, error, this->getVoxelStd(voxel), &voxel));
        }
        return stats;
    }

    std::vector<VoxelStatistics> stats(const TrueMap &trueMap, const std::vector<octomap::point3d> &positions) const
    {
        std::vector<VoxelStatistics> stats;
        for (auto &position : positions)
        {
            auto voxel = this->query(position);
            QTrueVoxel trueVoxel = trueMap.query(voxel.key);
            double trueMean = trueMap.getVoxelMean(trueVoxel);
            if (voxel.type != GEOMETRY_VOXEL)
            {
//                ROS_WARN_STREAM("Skipped stats computation for a voxel in "
//                                        << mapType() << " at position: " << voxel.position);
                double bernoulliStd = std::sqrt(Parameters::priorMean * (1. - Parameters::priorMean));
                stats.push_back(VoxelStatistics(trueMean > 0.5, Parameters::priorMean,
                                                Parameters::priorMean, bernoulliStd, &voxel));
                continue;
            }
            double mean = this->getVoxelMean(voxel);
            double error = (trueMean - mean);
            stats.push_back(VoxelStatistics(trueMean > 0.5, mean, error, this->getVoxelStd(voxel), &voxel));
        }
        return stats;
    }

    std::vector<VoxelStatistics> statsUpdated(const TrueMap &trueMap) const
    {
        std::vector<VoxelStatistics> stats;
        for (auto &voxel : updatedVoxels())
        {
            QTrueVoxel trueVoxel = trueMap.query(voxel.key);
            double trueMean = trueMap.getVoxelMean(trueVoxel);
            if (voxel.type != GEOMETRY_VOXEL)
            {
//                ROS_WARN_STREAM("Skipped stats computation for a voxel in "
//                                        << mapType() << " at position: " << voxel.position);
                double bernoulliStd = std::sqrt(Parameters::priorMean * (1. - Parameters::priorMean));
                stats.push_back(VoxelStatistics(trueMean > 0.5, Parameters::priorMean,
                                                Parameters::priorMean, bernoulliStd, &voxel));
                continue;
            }
            double mean = this->getVoxelMean(voxel);
            double error = (trueMean - mean);
            stats.push_back(VoxelStatistics(trueMean > 0.5, mean, error, this->getVoxelStd(voxel), &voxel));
        }
        return stats;
    }

    virtual std::vector<QTypedVoxel<NODE> > updatedVoxels() const = 0;

    std::vector<octomap::OcTreeKey> updatedKeys() const
    {
        std::vector<octomap::OcTreeKey> keys;
        for (auto &v : updatedVoxels())
            keys.push_back(v.key);
        return keys;
    }

    std::vector<octomap::point3d> updatedPositions() const
    {
        std::vector<octomap::point3d> positions;
        for (auto &v : updatedVoxels())
            positions.push_back(v.position);
        return positions;
    }
    virtual std::string mapType() const = 0;

protected:
    StatisticsMap(const octomap::OcTreeBaseImpl<NODE, I> *tree) : QVoxelMap<NODE, I>(tree)
    {}
};