#pragma once

#include "TrueMap.h"
#include "BeliefMap.h"
#include "LogOddsMap.h"
#include "../trajopt/BSplineTrajectory.h"
#include "FakeRobot.hpp"
#include "PixelSensor.h"
#include "Observation.hpp"
#include "Statistics.hpp"

class TrajectoryPlanner : public Observable
{
public:
    TrajectoryPlanner(TrueMap &trueMap, BeliefMap &beliefMap, LogOddsMap &logOddsMap);

    BSplineTrajectory replan(Point start, Point end, double startVelocity = 0.0);

    static BSplineTrajectory generateInitialDirectTrajectory(Point start, Point end);

    static std::vector<BSplineTrajectory> generateTrajectories();
    static std::vector<BSplineTrajectory> generateTrajectories(Point start, Point end,
                                                 VelocityPlanningParameters parameters = VelocityPlanningParameters());

    double evaluate(BSplineTrajectory &trajectory, BeliefMap &map, const smap::smapStats &stats);

    BSplineTrajectory currentEvaluationCandidate() const
    {
        return _candidate;
    }

private:
    TrueMap &_trueMap;
    BeliefMap &_beliefMap;
    BeliefMap _imaginaryMap;
    LogOddsMap &_logOddsMap;
    BSplineTrajectory _candidate;
    Statistics<FakeRobot<PixelSensor> > _stats;
    FakeRobot<PixelSensor> *_simulationBot;

    void _handleObservation(const Observation &observation);
};


