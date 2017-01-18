#pragma once

#include "TrueMap.h"
#include "BeliefMap.h"
#include "Trajectory.h"

class TrajectoryPlanner
{
public:
    TrajectoryPlanner(TrueMap &trueMap, BeliefMap &beliefMap);

    Trajectory replan(Point start, Point end);
    Trajectory replan(Point start, Point end, const Trajectory &basis);


    std::vector<Trajectory> generateTrajectories() const;

    void evaluate(Trajectory &trajectory, BeliefMap &map, const smap::smapStats &stats);

private:
    TrueMap &_trueMap;
    BeliefMap &_beliefMap;

};


