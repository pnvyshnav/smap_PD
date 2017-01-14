#pragma once

#include <ros/publisher.h>
#include <ros/node_handle.h>
#include <smap/smapStats.h>

#include "TrueMap.h"
#include "LogOddsMap.h"
#include "BeliefMap.h"
#include "FakeRobot.hpp"
#include "QVoxel.hpp"

class QVoxel;
class TrueMap;
struct VoxelStatistics
{
    double absError;
    double stdDev;
    const QVoxel *voxel;
    VoxelStatistics(double absError, double stdDev, const QVoxel *voxel)
            : absError(absError), stdDev(stdDev), voxel(voxel)
    {}
};


class MapStatistics
{
public:
    template <class NODE, class I>
    static std::vector<VoxelStatistics> stats(const TrueMap &trueMap, const QVoxelMap<NODE, I> &map)
    {
        std::vector<VoxelStatistics> stats;
        for (auto &voxel : map.voxels())
        {
            QTrueVoxel trueVoxel = trueMap.query(voxel.key);
            double error = std::abs(trueMap.getVoxelMean(trueVoxel) - map.getVoxelMean(voxel));
            stats.push_back(VoxelStatistics(error, map.getVoxelStd(voxel), &voxel));
        }
        return stats;
    }

    template <class NODE, class I>
    static std::vector<VoxelStatistics> stats(const TrueMap &trueMap, const QVoxelMap<NODE, I> &map, const Parameters::KeySet &keys)
    {
        std::vector<VoxelStatistics> stats;
        for (auto &voxel : map.voxels(keys))
        {
            QTrueVoxel trueVoxel = trueMap.query(voxel.key);
            double error = std::abs(trueMap.getVoxelMean(trueVoxel) - map.getVoxelMean(voxel));
            stats.push_back(VoxelStatistics(error, map.getVoxelStd(voxel), &voxel));
        }
        return stats;
    }
};

class Statistics
{
public:
    Statistics(const TrueMap &trueMap);
    ~Statistics();

    void update(const LogOddsMap &logOddsMap,
                const BeliefMap &beliefMap,
                const FakeRobot<> &robot);

    void saveToFile(std::string filename = "statistics.bag") const;

    /**
     * Computes the Pearson Correlation Coefficient between X and Y.
     * @param xs Array X.
     * @param ys Array Y.
     * @return Pearson Correlation Coefficient.
     */
    static double pcc(const std::vector<double> &xs, const std::vector<double> &ys);

    static double avg_distance(const std::vector<double> &xs, const std::vector<double> &ys);

    void reset();

private:
    ros::Publisher _publisher;
    ros::NodeHandle *_nh;
    smap::smapStats _msg;
    const TrueMap &_trueMap;
};
