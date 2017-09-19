#pragma once

#include <nlopt.hpp>
#include <ecl/time/stopwatch.hpp>

#include "TrueMap.h"
#include "BeliefMap.h"
#include "LogOddsMap.h"
#include "FakeRobot.hpp"
#include "PixelSensor.h"
#include "Observation.hpp"
#include "../trajopt/MinSnapTrajectory.h"
#include "../trajopt/BSplineTrajectory.h"

template<class TRAJ = MinSnapTrajectory>
class TrajectoryData
{
public:
    std::vector<double> reachabilities;
    std::vector<double> variances;

    std::vector<double> historicalReachabilities;
    double currentReachability;
    std::vector<double> imaginaryFutureReachabilities;
    std::vector<double> beliefFutureReachabilities;
    double arcLength;
    std::vector<Point> positions;
    std::vector<Point> velocities;
    std::vector<Point> accelerations;
    std::vector<double> centripetalForces;
    TRAJ &_trajectory;

    TrajectoryData(TRAJ &trajectory) : _trajectory(trajectory)
    {}

    void clear()
    {
        reachabilities.clear();
        variances.clear();
        historicalReachabilities.clear();
        imaginaryFutureReachabilities.clear();
        beliefFutureReachabilities.clear();
        positions.clear();
        velocities.clear();
        accelerations.clear();
        centripetalForces.clear();
    }
};

template<class TRAJ>
class TrajectoryCostFunction
{
public:
    virtual double operator()(const TrajectoryData<TRAJ> &data) = 0;
};

template<class TRAJ>
class PureReachability : public TrajectoryCostFunction<TRAJ>
{
public:
    /**
     * Computes product integral over reachabilities.
     * @param data Data collected during simulation of trajectory.
     * @return Negative reachability (for minimization).
     */
    double operator()(const TrajectoryData<TRAJ> &data)
    {
        double reach = 1;
        double arcLength = 0;
        for (unsigned int i = 1; i < data.reachabilities.size()-1; ++i)
        {
            double dist = data.positions[i].dist(data.positions[i-1]);
            reach *= std::pow(data.reachabilities[i], dist);
            arcLength += dist;
        }
//        reach = std::pow(reach, 1. / arcLength);
//        ROS_INFO("Reachability: %f", reach);
        return -reach;
    }
};

template<class TRAJ>
class MinimumVariance : public TrajectoryCostFunction<TRAJ>
{
public:
    /**
     * Computes product integral over reachabilities.
     * @param data Data collected during simulation of trajectory.
     * @return Negative reachability (for minimization).
     */
    double operator()(const TrajectoryData<TRAJ> &data)
    {
        double reach = 1;
        double arcLength = 0;

        double varLeft = 1, varRight = 1;

        for (unsigned int i = 1; i < data.reachabilities.size()-1; ++i)
        {
            double dist = data.positions[i].dist(data.positions[i-1]);
            reach *= std::pow(data.reachabilities[i], dist);
            arcLength += dist;

//            varLeft *= data.variances[i] + std::pow(data.reachabilities[i], 2.);
//            varRight *= std::pow(data.reachabilities[i], 2.);
            varLeft *= std::pow(data.variances[i] + std::pow(data.reachabilities[i], 2.), dist);
            varRight *= std::pow(data.reachabilities[i], 2. * dist);
//            ROS_INFO("VAR[i] = %f", data.variances[i]);
        }
        reach = std::pow(reach, 1. / arcLength);
//        ROS_INFO("Reachability: %f", reach);

        double ucb = - std::sqrt(varLeft-varRight);
        ROS_INFO("VAR: %f   MEAN: %f   UCB: %f", std::sqrt(varLeft-varRight), reach, ucb);
        return -ucb;
    }
private:
    const double kappa = 0.05;
};

template<class TRAJ>
class LowerConfidenceBound : public TrajectoryCostFunction<TRAJ>
{
public:
    /**
     * Computes product integral over reachabilities.
     * @param data Data collected during simulation of trajectory.
     * @return Negative reachability (for minimization).
     */
    double operator()(const TrajectoryData<TRAJ> &data)
    {
        double reach = 1;
        double arcLength = 0;

        double varLeft = 1, varRight = 1;
        double var = 1.;

        for (unsigned int i = 1; i < data.reachabilities.size()-1; ++i)
        {
            if (data.positions[i].x < Parameters::xMin || data.positions[i].x > Parameters::xMax ||
                data.positions[i].y < Parameters::yMin || data.positions[i].y > Parameters::yMax)
            {
//            ROS_WARN("VARIANCE OUT OF BOUNDS!");
                return 1000; // very bad cost
            }
            double dist = data.positions[i].dist(data.positions[i-1]);
//            reach *= std::pow(data.reachabilities[i] / (data.variances[i] / 1.1), dist);

            reach *= std::pow(data.reachabilities[i], dist);
//            reach *= data.reachabilities[i];
            arcLength += dist;

//            varLeft *= data.variances[i] + std::pow(data.reachabilities[i], 2.);
//            varRight *= std::pow(data.reachabilities[i], 2.);
            varLeft *= std::pow(data.variances[i] + std::pow(data.reachabilities[i], 2.), dist);
            varRight *= std::pow(data.reachabilities[i], 2. * dist);
//            ROS_INFO("VAR[i] = %f", data.variances[i]);

            var *= std::pow(data.variances[i], dist);
        }
        double std = std::sqrt(var);
//        reach = std::pow(reach, 1. / arcLength);
//        ROS_INFO("Reachability: %f", reach);

        std = std::sqrt(varLeft-varRight);
        double lcb = reach - _kappa * std; //-kappa*std::sqrt(varLeft-varRight); //reach - kappa * std::sqrt(varLeft-varRight);
//        ROS_INFO("VAR: %f   MEAN: %f   UCB: %f", std::sqrt(varLeft-varRight), reach, lcb);
//        ROS_INFO("STD: %f   MEAN: %f   L: %f   LCB: %f", std, reach, arcLength, lcb);
        return -lcb;
    }

    double kappa() const
    {
        return _kappa;
    }

    void setKappa(double kappa)
    {
        _kappa = kappa;
    }
private:
    double _kappa = 1.2;
};

template<class TRAJ>
class DeltaReach : public TrajectoryCostFunction<TRAJ>
{
public:
    /**
     * Computes change in reachabilities.
     * @param data Data collected during simulation of trajectory.
     * @return Negative reachability (for minimization).
     */
    double operator()(const TrajectoryData<TRAJ> &data)
    {
        double reach = 1;
        double arcLength = 0;

        double varLeft = 1, varRight = 1;
        double var = 1.;

        double deltaReach = 0;
        for (unsigned int i = 1; i < data.reachabilities.size(); ++i)
        {
            if (data.positions[i].x < Parameters::xMin || data.positions[i].x > Parameters::xMax ||
                data.positions[i].y < Parameters::yMin || data.positions[i].y > Parameters::yMax)
            {
//            ROS_WARN("VARIANCE OUT OF BOUNDS!");
                return 1000; // very bad cost
            }
            double dist = data.positions[i].dist(data.positions[i-1]);
//            reach *= std::pow(data.reachabilities[i] / (data.variances[i] / 1.1), dist);
            reach *= std::pow(data.reachabilities[i], dist);
            arcLength += dist;

//            varLeft *= data.variances[i] + std::pow(data.reachabilities[i], 2.);
//            varRight *= std::pow(data.reachabilities[i], 2.);
            varLeft *= std::pow(data.variances[i] + std::pow(data.reachabilities[i], 2.), dist);
            varRight *= std::pow(data.reachabilities[i], 2. * dist);
//            ROS_INFO("VAR[i] = %f", data.variances[i]);

            var *= std::pow(data.variances[i], dist);

            deltaReach += dist * (data.reachabilities[i-1] - data.reachabilities[i]);
        }
        double std = std::sqrt(var);

        ROS_INFO("DELTA REACH: %f   REACH: %f", deltaReach, reach);
        deltaReach -= 0.5 * reach;
//        reach = std::pow(reach, 1. / arcLength);
//        ROS_INFO("Reachability: %f", reach);

//        std = std::sqrt(varLeft-varRight);
        return deltaReach;
    }
};

/**
 * Trajectory optimizer.
 * @tparam TRAJ Type of trajectory.
 * @tparam SENSOR Type of robot sensor.
 * @tparam COST Type of cost function.
 */
template<class TRAJ, class SENSOR, class COST>
class TrajOpt;

/**
 * Minimum objective function for trajectory optimization.
 * @tparam TRAJ Type of trajectory.
 * @tparam SENSOR Type of robot sensor.
 * @tparam COST Type of cost function.
 */
template<class TRAJ, class SENSOR, class COST>
class TrajOptObjective
{
    friend class TrajOpt<TRAJ, SENSOR, COST>;
private:
    TrajOpt<TRAJ, SENSOR, COST> *_trajOpt;
    TrajOptObjective(TrajOpt<TRAJ, SENSOR, COST> *trajOpt)
        : _trajOpt(trajOpt)
    {
    }

public:
    double operator()(const std::vector<double> &x)
    {
        Eigen::VectorXd p = stdToEigenVector(x);
        _trajOpt->_candidate.parameterize(p);
        auto data = _trajOpt->_simulateCandidate();
        return _trajOpt->_cost(data);
    }

    static inline Eigen::VectorXd stdToEigenVector(const std::vector<double> &v)
    {
        Eigen::VectorXd e(v.size());
        for (int i = 0; i < v.size(); ++i)
            e(i) = v[i];
        return e;
    }
};

/**
 * Trajectory optimizer.
 * @tparam TRAJ Type of trajectory.
 * @tparam SENSOR Type of robot sensor.
 * @tparam COST Type of cost function.
 */
template<class TRAJ = MinSnapTrajectory, class SENSOR = StereoCameraSensor, class COST = LowerConfidenceBound<TRAJ> >
class TrajOpt : public Observable
{
    friend class TrajOptObjective<TRAJ, SENSOR, COST>;
public:
    TrajOpt(const TRAJ &trajectory, TrueMap *trueMap, BeliefMap *beliefMap)
        : _trajectory(trajectory), _trueMap(trueMap), _beliefMap(beliefMap),
          _objective(this), _data(_trajectory), _replanningIterations(0)
    {
    }

    TRAJ &currentEvaluationCandidate()
    {
        return _candidate;
    }

    TRAJ optimize(double absLimit = 400)
    {
        _candidate = _trajectory;
        /**
         * Non-deterministic methods (all?):
         * - LN_NEWUOA
         * - LN_BOBYQA
         * - LN_COBYLA
         * - LN_NELDERMEAD
         */
        nlopt::opt opt(nlopt::LN_COBYLA, _trajectory.dof());
        _optimizationStep = 0;

        std::vector<double> lb(_trajectory.dof(), -absLimit), ub(_trajectory.dof(), absLimit);
        opt.set_lower_bounds(lb);
        opt.set_upper_bounds(ub);

        opt.set_min_objective(TrajOpt::_wrap, &_objective);

        const double xtol_rel      = 1e-3; // 1e-5;
        const double xtol_abs      = 1e-2; // 1e-3;
        const unsigned int maxeval = 350; // 300;

        opt.set_xtol_rel(xtol_rel);
        // deactivate relative tolerance stopping criterion
//        opt.set_xtol_rel(-1.0);
        // stop when an optimization step changes all parameters by less than this value
        opt.set_xtol_abs(xtol_abs);
        // stop after so many iterations
        opt.set_maxeval(maxeval);

        std::vector<double> x(_trajectory.dof(), 0.);
        double minf;
        ecl::StopWatch stopWatch;
        nlopt::result result = opt.optimize(x, minf);
        if (result > 0)
        {
            ROS_INFO("Optimization finished successfully after %d steps. Best performance: %f. Stopping criterion: %d",
                     _optimizationStep+1, minf, result);
            _trajectory = _candidate;
        }
        else
        {
            ROS_INFO("Optimization failed after %d steps. Performance: %f. Stopping criterion: %d",
                     _optimizationStep+1, minf, result);
        }
        ROS_INFO_STREAM("Elapsed time for optimization: " << stopWatch.elapsed() << " seconds");

        return _candidate;
    }

    TRAJ replan(Point xStart, Point xEnd, Point xdStart)
    {
        ++_replanningIterations;
        int times = 6; //std::max(4, std::min(10, (int)std::round(xStart.dist(xEnd) * 6.)));
        int degree = 7; //std::max(4, std::min(7, (int)std::round(xStart.dist(xEnd) * 6.)));
        double absLimit = 600; //std::max(350., std::min(400., xStart.dist(xEnd) * 400.));
        ROS_INFO("REPLANNING with a %i time, %i degree piece polynomial. xStart.dist(xEnd): %f",
                 times, degree, xStart.dist(xEnd));
        // TODO implement this not only for Min Snap
        auto t = MinSnapTrajectory(xStart, xEnd, xdStart, Point(), // point to the left
                                   Point(), Point(),
        times, degree);
        _trajectory = t;
        optimize(absLimit);
        return _candidate;
    }

    COST &costFunction()
    {
        return _cost;
    }

    int replanningIterations() const
    {
        return _replanningIterations;
    }

private:
    TrueMap *_trueMap;
    BeliefMap *_beliefMap;
    BeliefMap _imaginaryMap;
    FakeRobot<SENSOR> *_simulationBot;

    int _optimizationStep;
    int _replanningIterations;

    TrajOptObjective<TRAJ, SENSOR, COST> _objective;

    TRAJ _trajectory;
    TRAJ _candidate;

    COST _cost;

    TrajectoryData<TRAJ> _data;

    void _handleObservation(const Observation &observation)
    {
//        ROS_INFO("SimulationBot position: %f %f %f", _simulationBot->position().x(), _simulationBot->position().y(), _simulationBot->position().z());
        _imaginaryMap.update(observation, *_trueMap);

        Point p = Point(_simulationBot->position().x(), _simulationBot->position().y()
#if (DIMENSIONS == 3)
                , _simulationBot->position().z()
#endif
        );
        _data.positions.push_back(p);

#if (DIMENSIONS == 3)
        _data.reachabilities.push_back(_beliefMap->filteredReachability(p.x, p.y, p.z));
#else
        _data.reachabilities.push_back(_beliefMap->filteredReachability(p.x, p.y, Parameters::voxelSize/2.));
#endif
#if (DIMENSIONS == 3)
        _data.variances.push_back(_imaginaryMap.filteredVariance(p.x, p.y, p.z));
#else
        _data.variances.push_back(_imaginaryMap.filteredVariance(p.x, p.y, Parameters::voxelSize/2.));
#endif

        // look into the future
//        _data.beliefFutureReachabilities.clear();
//        _data.imaginaryFutureReachabilities.clear();
//        for (unsigned int step = _simulationBot->currentStep(); step < Parameters::FakeRobotNumSteps; ++step)
//        {
//            double u = step * 1. / Parameters::FakeRobotNumSteps;
//            auto r = _trajectory.evaluate(u);
//            auto x = r.point;
//            _data.beliefFutureReachabilities.push_back(_beliefMap.filteredReachability(x.x, x.y, Parameters::voxelSize/2.));
//            _data.imaginaryFutureReachabilities.push_back(_imaginaryMap.filteredReachability(x.x, x.y, Parameters::voxelSize/2.));
//        }
    }

    const BeliefMap& _constructImaginaryMap()
    {
        _imaginaryMap = BeliefMap(*_beliefMap);
//        auto obstacle1 = std::vector<double> {
//                -1, -1, // left bottom
//                -.1, -.3 // right top
//        };
//        auto obstacle2 = std::vector<double> {
//                -1, .3, // left bottom
//                -.1, 1 // right top
//        };
//        double smoothness = 1.9 * Parameters::voxelSize;
//        double certainty = 0.7;
//        double prior = 0.5;
        for (unsigned int x = 0; x < Parameters::voxelsPerDimensionX; ++x)
        {
            for (unsigned int y = 0; y < Parameters::voxelsPerDimensionY; ++y)
            {
                for (unsigned int z = 0; z < Parameters::voxelsPerDimensionZ; ++z)
                {
                    double _x = Parameters::xMin + x * Parameters::voxelSize;
                    double _y = Parameters::yMin + y * Parameters::voxelSize;
                    double _z = Parameters::zMin + z * Parameters::voxelSize;
//                    _imaginaryMap.belief(_imaginaryMap.coordToKey(_x, _y, _z))->lockMean();
                    auto belief = _imaginaryMap.belief(_imaginaryMap.coordToKey(_x, _y, _z));
                    // lock mean & variance where we took measurements
                    if (std::abs(belief->mean() - Parameters::priorMean) > 1e-2)
                    {
                        _imaginaryMap.belief(_imaginaryMap.coordToKey(_x, _y, _z))->storeMeanVariance(
                                belief->mean(), belief->variance()
                        );
                    }
                    continue;
//                    double mean, variance;
//                    if (((obstacle1[0] <= _x && _x <= obstacle1[2]) && (obstacle1[1] <= _y && _y <= obstacle1[3])) ||
//                        ((obstacle2[0] <= _x && _x <= obstacle2[2]) && (obstacle2[1] <= _y && _y <= obstacle2[3])))
//                    {
//                        mean = prior + certainty * (1 - prior);
//                        variance = mean * (1. - mean);
//                    }
//                    else
//                    {
//                        double rx1 = .5 * (obstacle1[0] + obstacle1[2]);
//                        double ry1 = .5 * (obstacle1[1] + obstacle1[3]);
//                        double rw1 = obstacle1[2] - obstacle1[0];
//                        double rh1 = obstacle1[3] - obstacle1[1];
//
//                        double dx1 = std::max(std::abs(_x - rx1) - rw1 / 2, 0.);
//                        double dy1 = std::max(std::abs(_y - ry1) - rh1 / 2, 0.);
//                        double d1 = std::sqrt(std::pow(dx1, 2.) + std::pow(dy1, 2.));
//
//                        double rx2 = .5 * (obstacle2[0] + obstacle2[2]);
//                        double ry2 = .5 * (obstacle2[1] + obstacle2[3]);
//                        double rw2 = obstacle2[2] - obstacle2[0];
//                        double rh2 = obstacle2[3] - obstacle2[1];
//
//                        double dx2 = std::max(std::abs(_x - rx2) - rw2 / 2, 0.);
//                        double dy2 = std::max(std::abs(_y - ry2) - rh2 / 2, 0.);
//                        double d2 = std::sqrt(std::pow(dx2, 2.) + std::pow(dy2, 2.));
//
//                        double n = prior;
//                        double m = -n / smoothness;
//                        double truth = std::max(m * (std::min(d1, d2) - smoothness) + n, 0.);
//                        mean = (truth - prior) * certainty + prior;
//                        variance = mean * (1. - mean);
//                    }
//
//                    _imaginaryMap.belief(_imaginaryMap.coordToKey(_x, _y, _z))->storeMeanVariance(mean, variance);
                }
            }
        }
        return _imaginaryMap;
    }

    const TrajectoryData<TRAJ> _simulateCandidate()
    {
        _data.clear();

        if (_optimizationStep % 10 == 0)
            updateSubscribers();

        _imaginaryMap = BeliefMap(*_beliefMap);
        _constructImaginaryMap();

//        auto further = candidate.evaluate(0.4);


//        _simulationBot = new FakeRobot<PixelSensor>(Parameters::Vec3Type((float)further.point.x,(float)further.point.y, 0.05),
//                                             Parameters::Vec3Type(1, 0, 0),
//                                             _trueMap, _imaginaryMap);
//        ROS_INFO("Candidate start: %f %f", _candidate.start().x, _candidate.start().y);
//        ROS_INFO("Trajectory(0): %f %f !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!",
//                 _candidate.evaluate(0).point.x, _candidate.evaluate(0).point.y);
        _simulationBot = new FakeRobot<SENSOR>(Parameters::Vec3Type((float)_candidate.start().x,
                                                                    (float)_candidate.start().y,
                                                                    0.05),
                                               Parameters::Vec3Type(0, 1, 0),
                                               *_trueMap, _imaginaryMap);
        _simulationBot->setObservationMode(OBSERVE_BOTH);
        _simulationBot->registerObserver(std::bind(&TrajOpt::_handleObservation,
                                                   this,
                                                   std::placeholders::_1));

        _simulationBot->setTrajectory(_candidate);
        _simulationBot->run();
        delete _simulationBot;
        ++_optimizationStep;

        return _data;
    }

    static double _wrap(const std::vector<double> &x, std::vector<double> &grad, void *data)
    {
        return (*reinterpret_cast<TrajOptObjective<TRAJ, SENSOR, COST>* >(data))(x);
    }

};

typedef TrajOpt<> TrajectoryPlanner;
