#pragma once

#include <ros/publisher.h>
#include <ros/node_handle.h>
#include <smap/smapStats.h>

#include "TrueMap.h"
#include "LogOddsMap.h"
#include "BeliefMap.h"

class Statistics
{
public:
    Statistics(const TrueMap &trueMap);
    ~Statistics();

    void update(const LogOddsMap &logOddsMap, const BeliefMap &beliefMap);

    void saveToFile(std::string filename = "statistics.bag") const;

private:
    ros::Publisher _publisher;
    ros::NodeHandle *_nh;
    smap::smapStats _msg;
    const TrueMap _trueMap;
    unsigned int _step;
};
