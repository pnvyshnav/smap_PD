#pragma once

#include <ros/publisher.h>
#include <ros/node_handle.h>
#include <smap/smapStats.h>
#include <rosbag/bag.h>

#include <algorithm>
#include <iterator>

#include "TrueMap.h"
#include "LogOddsMap.h"
#include "BeliefMap.h"
#include "FakeRobot.hpp"


// TODO revert to Stereo Camera sensor!
template<class ROBOT=FakeRobot<StereoCameraSensor> >
class Statistics
{
public:
    Statistics(const TrueMap &trueMap) : _trueMap(trueMap), _step(0)
    {
#ifdef PUBLISH_STATS
        _nh = new ros::NodeHandle;
        _publisher = _nh->advertise<smap::smapStats>("stats", 1);
#endif
    }

    ~Statistics()
    {
#ifdef PUBLISH_STATS
        delete _nh;
#endif
    }

    void update(const LogOddsMap &logOddsMap,
                const BeliefMap &beliefMap,
                GaussianProcessMap &gpMap,
                ROBOT &robot)
    {
#ifdef REAL_3D
        _msg.step = _step;
        _msg.maxStep = _msg.step;
#else
        _msg.step = robot.currentStep() + 1;
        _msg.maxStep = Parameters::FakeRobotNumSteps;
#endif
        _step++;
        _robot = &robot;

        auto beliefCompleteStats = beliefMap.stats(_trueMap);
        auto logOddsCompleteStats = logOddsMap.stats(_trueMap);
        auto gpCompleteStats = gpMap.stats(_trueMap);

        auto beliefUpdatedPositions = beliefMap.updatedPositions();
        auto logOddsUpdatedPositions = logOddsMap.updatedPositions();
        std::vector<octomap::point3d> updatedPositions;
        for (auto &position : beliefUpdatedPositions)
        {
            if (std::find(logOddsUpdatedPositions.begin(), logOddsUpdatedPositions.end(), position) != logOddsUpdatedPositions.end())
            {
                updatedPositions.push_back(position);
#ifndef MANY_STEPS
                _msg.updatedVoxelsX.push_back(position.x());
                _msg.updatedVoxelsY.push_back(position.y());
                _msg.updatedVoxelsZ.push_back(position.z());
#endif
            }
        }

//    ROS_INFO("Selected %d updated keys (%d from BeliefMap / %d from LogOddsMap are missing)",
//             (int)updatedPositions.size(),
//             (int)(beliefUpdatedPositions.size() - updatedPositions.size()),
//             (int)(logOddsUpdatedPositions.size() - updatedPositions.size()));

        auto beliefUpdatedStats = beliefMap.stats(_trueMap, updatedPositions);
        auto logOddsUpdatedStats = logOddsMap.stats(_trueMap, updatedPositions);

        _msg.errorLogOdds = VoxelStatistics::selectError(logOddsCompleteStats);
        _msg.errorBelief = VoxelStatistics::selectError(beliefCompleteStats);
        _msg.errorGP = VoxelStatistics::selectError(gpCompleteStats);

        _msg.stdLogOdds = VoxelStatistics::selectStd(logOddsCompleteStats);
        _msg.stdBelief = VoxelStatistics::selectStd(beliefCompleteStats);
        _msg.stdGP = VoxelStatistics::selectStd(gpCompleteStats);

        assert(_msg.errorLogOdds.size() == _msg.errorBelief.size());

        _msg.voxels = _msg.errorBelief.size();

        auto errorUpdatedBelief = VoxelStatistics::selectError(beliefUpdatedStats);
        auto errorUpdatedLogOdds = VoxelStatistics::selectError(logOddsUpdatedStats);
        assert(errorUpdatedBelief.size() == errorUpdatedLogOdds.size());
        _msg.updatedVoxels.push_back(beliefMap.updatedVoxels().size());

        // XXX these assertions aren't necessary if we only consider intersection of updated voxels
        //assert(beliefMap.updatedVoxels().size() == logOddsMap.updatedVoxels().size());
        //assert(beliefMap.updatedVoxels().size() == errorUpdatedBelief.size());

        auto stdUpdatedBelief = VoxelStatistics::selectStd(beliefUpdatedStats);
        auto stdUpdatedLogOdds = VoxelStatistics::selectStd(logOddsUpdatedStats);

        std::vector<double> absErrorBelief = _msg.errorBelief;
        std::vector<double> absErrorLogOdds = _msg.errorLogOdds;
        std::vector<double> absErrorGP = _msg.errorGP;
        for (unsigned int i = 0; i < absErrorBelief.size(); ++i)
        {
            absErrorBelief[i] = std::abs(absErrorBelief[i]);
            absErrorLogOdds[i] = std::abs(absErrorLogOdds[i]);
            absErrorGP[i] = std::abs(absErrorGP[i]);
        }
        _msg.stdErrorCorrelationBelief.push_back(pcc(_msg.stdBelief, absErrorBelief));
        _msg.stdErrorCorrelationLogOdds.push_back(pcc(_msg.stdLogOdds, absErrorLogOdds));
        _msg.stdErrorCorrelationGP.push_back(pcc(_msg.stdGP, absErrorGP));

#if !defined(MANY_STEPS) || defined(COMPUTE_UPDATED_EVOLUTION)
        // append current errors to complete error vectors
//        _msg.errorCompleteLogOdds.insert(_msg.errorCompleteLogOdds.end(),
//                                         _msg.errorLogOdds.begin(),
//                                         _msg.errorLogOdds.end());
//        _msg.errorCompleteBelief.insert(_msg.errorCompleteBelief.end(),
//                                        _msg.errorBelief.begin(),
//                                        _msg.errorBelief.end());

        // append current std devs to complete std dev vectors
//        _msg.stdCompleteLogOdds.insert(_msg.stdCompleteLogOdds.end(),
//                                       _msg.stdLogOdds.begin(),
//                                       _msg.stdLogOdds.end());
//        _msg.stdCompleteBelief.insert(_msg.stdCompleteBelief.end(),
//                                      _msg.stdBelief.begin(),
//                                      _msg.stdBelief.end());
#endif

#ifndef MANY_STEPS
        // append current errors of updated voxels to complete updated error vectors
        _msg.errorCompleteUpdatedLogOdds.insert(_msg.errorCompleteUpdatedLogOdds.end(),
                                                errorUpdatedLogOdds.begin(),
                                                errorUpdatedLogOdds.end());
        _msg.errorCompleteUpdatedBelief.insert(_msg.errorCompleteUpdatedBelief.end(),
                                               errorUpdatedBelief.begin(),
                                               errorUpdatedBelief.end());

        // append current std devs of updated voxels to complete updated error vectors
        _msg.stdCompleteUpdatedBelief.insert(_msg.stdCompleteUpdatedBelief.end(),
                                             stdUpdatedBelief.begin(),
                                             stdUpdatedBelief.end());
        _msg.stdCompleteUpdatedLogOdds.insert(_msg.stdCompleteUpdatedLogOdds.end(),
                                              stdUpdatedLogOdds.begin(),
                                              stdUpdatedLogOdds.end());
#else
        _msg.errorCompleteUpdatedBelief = errorUpdatedBelief;
        _msg.errorCompleteUpdatedLogOdds = errorUpdatedLogOdds;
        _msg.stdCompleteUpdatedBelief = stdUpdatedBelief;
        _msg.stdCompleteUpdatedLogOdds = stdUpdatedLogOdds;
#endif

        _msg.noiseStd = Parameters::sensorNoiseStd;

        _msg.ismIncrement = LogOddsMap::parameters.increment;
        _msg.ismRampSize = LogOddsMap::parameters.rampSize;
        _msg.ismTopSize = LogOddsMap::parameters.topSize;
        _msg.ismRampSlope = LogOddsMap::parameters.rampSlope;

        _msg.gpParameter1 = GaussianProcessMap::parameters.parameter1;
        _msg.gpParameter2 = GaussianProcessMap::parameters.parameter2;
        _msg.gpParameter3 = GaussianProcessMap::parameters.parameter3;
        _msg.kernel = GaussianProcessMap::parameters.kernel;

        // error evolution
        double evolutionLogOdds = 0;
        for (double error : _msg.errorLogOdds)
            evolutionLogOdds += std::round(std::abs(error) + 0.01);
//        evolutionLogOdds /= _msg.errorLogOdds.size();
        _msg.errorEvolutionLogOdds.push_back(evolutionLogOdds);
        double evolutionBelief = 0;
        for (double error : _msg.errorBelief)
            evolutionBelief += std::round(std::abs(error) + 0.01);
//        evolutionBelief /= _msg.errorBelief.size();
        _msg.errorEvolutionBelief.push_back(evolutionBelief);
        double evolutionGP = 0;
        for (double error : _msg.errorGP)
            evolutionGP += std::round(std::abs(error) + 0.01);
//        evolutionGP /= _msg.errorGP.size();
        _msg.errorEvolutionGP.push_back(evolutionGP);

        std::cout << "MAE Advantage over Log Odds: " << evolutionLogOdds-evolutionBelief << std::endl;

        _msg.stdLogOdds = VoxelStatistics::selectStd(logOddsCompleteStats);
        _msg.stdBelief = VoxelStatistics::selectStd(beliefCompleteStats);

        // std evolution
        evolutionLogOdds = 0;
        for (double std : _msg.stdLogOdds)
            evolutionLogOdds += std::abs(std);
        evolutionLogOdds /= _msg.stdLogOdds.size();
        _msg.stdEvolutionLogOdds.push_back(evolutionLogOdds);
        evolutionBelief = 0;
        for (double std : _msg.stdBelief)
            evolutionBelief += std::abs(std);
        evolutionBelief /= _msg.stdBelief.size();
        _msg.stdEvolutionBelief.push_back(evolutionBelief);

#ifdef STATS_PARTICLES
        auto particles = beliefMap.particles();
        _msg.particles.insert(_msg.particles.end(), particles.begin(), particles.end());
        auto beliefMeans = VoxelStatistics::selectMean(beliefCompleteStats);
        _msg.beliefMeans.insert(_msg.beliefMeans.end(), beliefMeans.begin(), beliefMeans.end());
        auto trueMeans = VoxelStatistics::selectTrueMean(beliefCompleteStats);
        _msg.trueMeans.insert(_msg.trueMeans.end(), trueMeans.begin(), trueMeans.end());
        auto beliefStds = VoxelStatistics::selectStd(beliefCompleteStats);
        _msg.beliefStds.insert(_msg.beliefStds.end(), beliefStds.begin(), beliefStds.end());
#endif

#ifndef MANY_STEPS
        // append current std devs to complete std dev vectors
//        _msg.stdCompleteLogOdds.insert(_msg.stdCompleteLogOdds.end(),
//                                       _msg.stdLogOdds.begin(),
//                                       _msg.stdLogOdds.end());
//        _msg.stdCompleteBelief.insert(_msg.stdCompleteBelief.end(),
//                                      _msg.stdBelief.begin(),
//                                      _msg.stdBelief.end());
#endif

#ifdef PLANNER_2D_TEST
        _msg.trajectoryVoxels = (unsigned int) robot.trajectory().splineVoxelKeys(_trueMap).size();
        //ROS_INFO("Trajectory voxels: %d", (int)_msg.trajectoryVoxels);
        _msg.trajectoryVoxelX.clear();
        _msg.trajectoryVoxelY.clear();
        for (auto &position: robot.trajectory().splineVoxelPositions(_trueMap))
        {
            _msg.trajectoryVoxelX.push_back(position.x());
            _msg.trajectoryVoxelY.push_back(position.y());
            auto beliefVoxel = beliefMap.query(position);
            _msg.trajectoryOccupanciesBelief.push_back(beliefMap.getVoxelMean(beliefVoxel));
            _msg.trajectoryStdDevsBelief.push_back(beliefMap.getVoxelStd(beliefVoxel));
            //auto logOddsVoxel = logOddsMap.query(position);
            //_msg.trajectoryOccupanciesLogOdds.push_back(logOddsMap.getVoxelMean(logOddsVoxel));
            //_msg.trajectoryStdDevsLogOdds.push_back(logOddsMap.getVoxelStd(logOddsVoxel));
        }

        // add trajectory position / angle
        _msg.trajectoryX.push_back(robot.position().x());
        _msg.trajectoryY.push_back(robot.position().y());
        _msg.trajectoryT.push_back(robot.yaw());
//    _msg.trajectoryId = robot.selectedSpline();

        _msg.trajectoryTime.push_back(robot.time());

        _msg.trajectoryFutureVoxels.push_back(robot.currentSplineFutureVoxels().size());
        double reachability = 1;
        for (auto &voxel : beliefMap.voxels(robot.currentSplineFutureVoxels()))
        {
            reachability *= 1. - beliefMap.getVoxelMean(voxel);
            _msg.trajectoryFutureOccupanciesBelief.push_back(beliefMap.getVoxelMean(voxel));
            _msg.trajectoryFutureStdDevsBelief.push_back(beliefMap.getVoxelStd(voxel));
        }
        _msg.trajectoryFutureReachability.push_back(reachability);
#endif

#ifdef PUBLISH_STATS
        _publisher.publish(_msg);

        // wait some time to ensure data is plotted
        ros::Rate publishing_rate(29);
        publishing_rate.sleep();
#endif
    }

    void saveToFile(std::string filename)
    {
        // update final stats for updated voxels
        _msg.errorFinalUpdatedBelief.clear();
        _msg.errorFinalUpdatedLogOdds.clear();
        _msg.stdFinalUpdatedBelief.clear();
        _msg.stdFinalUpdatedLogOdds.clear();
#ifdef PLANNER_2D_TEST
        _msg.trajectoryControlX.clear();
        _msg.trajectoryControlY.clear();
        for (auto &p : _robot->trajectory().controlPoints())
        {
            _msg.trajectoryControlX.push_back(p.x);
            _msg.trajectoryControlY.push_back(p.y);
        }
#endif
        // compute indices at last step where log odds and SMAP voxel occupancy != prior
        std::vector<unsigned int> updatedIndices;
        for (unsigned int i = 0; i < _msg.errorBelief.size(); ++i)
        {
            if (std::abs(_msg.errorBelief[i]-Parameters::priorMean) > Parameters::equalityThreshold
                && std::abs(_msg.errorLogOdds[i]-Parameters::priorMean) > Parameters::equalityThreshold)
            {
                updatedIndices.push_back(i);
                _msg.errorFinalUpdatedBelief.push_back(_msg.errorBelief[i]);
                _msg.errorFinalUpdatedLogOdds.push_back(_msg.errorLogOdds[i]);
                _msg.stdFinalUpdatedBelief.push_back(_msg.stdBelief[i]);
                _msg.stdFinalUpdatedLogOdds.push_back(_msg.stdLogOdds[i]);
            }
        }
        ROS_INFO("%d of %d voxels were updated by SMAP and Log Odds.",
                 (int)updatedIndices.size(), (int)_msg.errorBelief.size());

#ifdef COMPUTE_UPDATED_EVOLUTION
        _msg.errorEvolutionUpdatedBelief.clear();
        _msg.errorEvolutionUpdatedLogOdds.clear();
        for (unsigned int step = 0; step < _step; ++step)
        {
            double evolutionLogOdds = 0, evolutionBelief = 0;
            for (auto i : updatedIndices)
            {
                evolutionLogOdds += std::abs(_msg.errorCompleteLogOdds[step*_msg.voxels + i]);
                evolutionBelief += std::abs(_msg.errorCompleteBelief[step*_msg.voxels + i]);
            }
            evolutionLogOdds /= updatedIndices.size();
            evolutionBelief /= updatedIndices.size();
            _msg.errorEvolutionUpdatedLogOdds.push_back(evolutionLogOdds);
            _msg.errorEvolutionUpdatedBelief.push_back(evolutionBelief);
        }
#endif

#if defined(MANY_STEPS) && defined(COMPUTE_UPDATED_EVOLUTION)
        // back up complete errors and stds to clear the message
        auto completeErrorLogOdds = std::vector<double>(_msg.errorCompleteLogOdds);
        auto completeErrorBelief = std::vector<double>(_msg.errorCompleteBelief);
        _msg.errorCompleteLogOdds.clear();
        _msg.errorCompleteBelief.clear();
        auto completeStdLogOdds = std::vector<double>(_msg.stdCompleteLogOdds);
        auto completeStdBelief = std::vector<double>(_msg.stdCompleteBelief);
        _msg.stdCompleteLogOdds.clear();
        _msg.stdCompleteBelief.clear();
#endif

        rosbag::Bag bag;
        bag.open(filename, rosbag::bagmode::Write);
        //bag.setCompression(rosbag::compression::BZ2);
        bag.write("stats", ros::Time::now(), _msg);
        bag.close();
        ROS_INFO_STREAM("Saved statistics ROS bag file to " << filename);

#if defined(MANY_STEPS) && defined(COMPUTE_UPDATED_EVOLUTION)
        _msg.errorCompleteLogOdds = std::vector<double>(completeErrorLogOdds);
        _msg.errorCompleteBelief = std::vector<double>(completeErrorBelief);
        _msg.stdCompleteLogOdds = std::vector<double>(completeStdLogOdds);
        _msg.stdCompleteBelief = std::vector<double>(completeStdBelief);
#endif
    }

    /**
     * Computes the Pearson Correlation Coefficient between X and Y.
     * @param xs Array X.
     * @param ys Array Y.
     * @return Pearson Correlation Coefficient.
     */
    double pcc(const std::vector<double> &xs, const std::vector<double> &ys)
    {
        assert(xs.size() == ys.size());
        if (xs.size() != ys.size())
            ROS_ERROR("Error during PCC computation: input arrays have different lengths.");
        int n = (int) xs.size();
        double meanX = 0, meanY = 0;
        for (unsigned int i = 0; i < n; ++i)
        {
            meanX += xs[i];
            meanY += ys[i];
        }
        meanX /= n; meanY /= n;
        double stdX = 0, stdY = 0;
        for (unsigned int i = 0; i < n; ++i)
        {
            stdX += std::pow(xs[i] - meanX, 2.);
            stdY += std::pow(ys[i] - meanY, 2.);
        }
        stdX = std::sqrt(stdX / (n-1));
        stdY = std::sqrt(stdY / (n-1));

        double ssp = 0.; // standard score product for x and y
        for (unsigned int i = 0; i < n; ++i)
            ssp += ((xs[i]-meanX) / stdX) * ((ys[i]-meanY) / stdY);
        return ssp / (n - 1);
    }

    double avg_distance(const std::vector<double> &xs, const std::vector<double> &ys)
    {
        assert(xs.size() == ys.size());
        if (xs.size() != ys.size())
            ROS_ERROR("Error during AVG distance computation: input arrays have different lengths.");
        int n = (int) xs.size();
        double meanDst = 0.;
        for (unsigned int i = 0; i < n; ++i)
        {
            meanDst += std::abs(xs[i] - ys[i]);
        }
        meanDst /= n;
        return meanDst;
    }

    void registerStepTimeBelief(double time)
    {
        _msg.timeBelief.push_back(time);
    }

    void registerStepTimeLogOdds(double time)
    {
        _msg.timeLogOdds.push_back(time);
    }

    void registerStepTimeGP(double time)
    {
        _msg.timeGP.push_back(time);
    }

    void registerMeasurements(int measurements)
    {
        _msg.measurements.push_back(measurements);
    }

    void registerRayStatistics(double minLength, double maxLength, double avgLength)
    {
        _msg.minRayLength.push_back(minLength);
        _msg.maxRayLength.push_back(maxLength);
        _msg.avgRayLength.push_back(avgLength);
    }

    void reset()
    {
        _step = 0;
        _msg.step = 0;
        _msg.maxStep = 0;
        _msg.voxels = 0;

        _msg.timeBelief.clear();
        _msg.timeLogOdds.clear();
        _msg.timeGP.clear();
        _msg.measurements.clear();

        _msg.minRayLength.clear();
        _msg.maxRayLength.clear();
        _msg.avgRayLength.clear();

        _msg.errorBelief.clear();
        _msg.errorLogOdds.clear();
        _msg.errorGP.clear();
        _msg.errorEvolutionBelief.clear();
        _msg.errorEvolutionLogOdds.clear();
        _msg.errorEvolutionGP.clear();
        _msg.errorEvolutionUpdatedBelief.clear();
        _msg.errorEvolutionUpdatedLogOdds.clear();
        _msg.stdEvolutionBelief.clear();
        _msg.stdEvolutionLogOdds.clear();
        _msg.stdEvolutionGP.clear();

        _msg.errorFinalUpdatedBelief.clear();
        _msg.errorFinalUpdatedLogOdds.clear();
        _msg.stdFinalUpdatedBelief.clear();
        _msg.stdFinalUpdatedLogOdds.clear();

        _msg.errorCompleteBelief.clear();
        _msg.errorCompleteLogOdds.clear();

        _msg.updatedVoxels.clear();
        _msg.updatedVoxelsX.clear();
        _msg.updatedVoxelsY.clear();
        _msg.updatedVoxelsZ.clear();

        _msg.errorCompleteUpdatedBelief.clear();
        _msg.errorCompleteUpdatedLogOdds.clear();

        _msg.stdBelief.clear();
        _msg.stdLogOdds.clear();
        _msg.stdGP.clear();

        _msg.stdCompleteBelief.clear();
        _msg.stdCompleteLogOdds.clear();

        _msg.stdCompleteUpdatedBelief.clear();
        _msg.stdCompleteUpdatedLogOdds.clear();

        _msg.stdErrorCorrelationBelief.clear();
        _msg.stdErrorCorrelationLogOdds.clear();
        _msg.stdErrorCorrelationGP.clear();

        _msg.trajectoryVoxels = 0;
        _msg.trajectoryVoxelX.clear();
        _msg.trajectoryVoxelY.clear();

        _msg.trajectoryOccupanciesBelief.clear();
        _msg.trajectoryOccupanciesLogOdds.clear();
        _msg.trajectoryStdDevsBelief.clear();
        _msg.trajectoryStdDevsLogOdds.clear();


        _msg.trajectoryX.clear();
        _msg.trajectoryY.clear();
        _msg.trajectoryT.clear();
        _msg.trajectoryTime.clear();

        _msg.trajectoryFutureVoxels.clear();
        _msg.trajectoryFutureReachability.clear();
        _msg.trajectoryFutureOccupanciesBelief.clear();
        //_msg.trajectoryFutureOccupanciesLogOdds.clear();
        _msg.trajectoryFutureStdDevsBelief.clear();
        //_msg.trajectoryFutureStdDevsLogOdds.clear();
    }

    const smap::smapStats stats() const
    {
        return _msg;
    }

private:
    ros::Publisher _publisher;
    ros::NodeHandle *_nh;
    smap::smapStats _msg;
    const TrueMap &_trueMap;
    unsigned int _step;
    ROBOT *_robot;
};
