#pragma once

#include <ros/publisher.h>
#include <ros/node_handle.h>
#include <smap/smapStats.h>

#include "TrueMap.h"
#include "LogOddsMap.h"
#include "BeliefMap.h"
#include "FakeRobot.hpp"


class Statistics
{
public:
    Statistics(const TrueMap &trueMap);
    ~Statistics();

    void update(const LogOddsMap &logOddsMap,
                const BeliefMap &beliefMap,
                FakeRobot<> &robot);

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

    const smap::smapStats stats() const
    {
        return _msg;
    }

private:
    ros::Publisher _publisher;
    ros::NodeHandle *_nh;
    smap::smapStats _msg;
    const TrueMap &_trueMap;
};
