#pragma once

#include "TrueMap.h"
#include "BeliefMap.h"
#include "LogOddsMap.h"
#include "Trajectory.h"
#include "FakeRobot.hpp"
#include "PixelSensor.h"
#include "Observation.hpp"
#include "Statistics.h"

class TrajectoryPlanner : public Observable
{
public:
    TrajectoryPlanner(TrueMap &trueMap, BeliefMap &beliefMap, LogOddsMap &logOddsMap);

    Trajectory replan(Point start, Point end, double startVelocity = 0.0);

    static Trajectory generateInitialDirectTrajectory(Point start, Point end);

    static std::vector<Trajectory> generateTrajectories();
    static std::vector<Trajectory> generateTrajectories(Point start, Point end,
                                                 VelocityPlanningParameters parameters = VelocityPlanningParameters());

    double evaluate(Trajectory &trajectory, BeliefMap &map, const smap::smapStats &stats);

    Trajectory currentEvaluationCandidate() const
    {
        return _candidate;
    }

private:
    TrueMap &_trueMap;
    BeliefMap &_beliefMap;
    BeliefMap _imaginaryMap;
    LogOddsMap &_logOddsMap;
    Trajectory _candidate;
    Statistics<FakeRobot<PixelSensor> > _stats;
    FakeRobot<PixelSensor> *_simulationBot;

    void _handleObservation(const Observation &observation);
};


