#include "../include/TrueMap.h"
#include "../include/Statistics.h"

#include <smap/smapStats.h>
#include <rosbag/bag.h>

#include <algorithm>
#include <iterator>



Statistics::Statistics(const TrueMap &trueMap) : _trueMap(trueMap)
{
    _nh = new ros::NodeHandle;
    _publisher = _nh->advertise<smap::smapStats>("stats", 1);
}

Statistics::~Statistics()
{
    delete _nh;
}

template<class IN, class OUT>
std::vector<OUT> convertVector(const std::vector<IN> &input)
{
    std::vector<OUT> output(input.begin(), input.end());
    return output;
};

void Statistics::update(const LogOddsMap &logOddsMap, const BeliefMap &beliefMap, FakeRobot<> &robot)
{
    _msg.step = robot.currentStep();
    _msg.maxStep = Parameters::FakeRobotNumSteps;

    auto beliefCompleteStats = beliefMap.stats(_trueMap);
    auto logOddsCompleteStats = logOddsMap.stats(_trueMap);

    auto beliefUpdatedPositions = beliefMap.updatedPositions();
    auto logOddsUpdatedPositions = logOddsMap.updatedPositions();
    std::vector<octomap::point3d> updatedPositions;
    for (auto &key : beliefUpdatedPositions)
    {
        if (std::find(logOddsUpdatedPositions.begin(), logOddsUpdatedPositions.end(), key) != logOddsUpdatedPositions.end())
        {
            updatedPositions.push_back(key);
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

    _msg.stdLogOdds = VoxelStatistics::selectStd(logOddsCompleteStats);
    _msg.stdBelief = VoxelStatistics::selectStd(beliefCompleteStats);

    assert(_msg.errorLogOdds.size() == _msg.errorBelief.size());

    _msg.voxels = _msg.errorBelief.size();

    auto errorUpdatedBelief = VoxelStatistics::selectError(beliefUpdatedStats);
    auto errorUpdatedLogOdds = VoxelStatistics::selectError(logOddsUpdatedStats);
    assert(errorUpdatedBelief.size() == errorUpdatedLogOdds.size());
    _msg.updatedVoxels.push_back(beliefMap.updatedVoxels().size());

    // TODO these assertions aren't necessary if we only consider intersection of updated voxels
    //assert(beliefMap.updatedVoxels().size() == logOddsMap.updatedVoxels().size());
    //assert(beliefMap.updatedVoxels().size() == errorUpdatedBelief.size());

    auto stdUpdatedBelief = VoxelStatistics::selectStd(beliefUpdatedStats);
    auto stdUpdatedLogOdds = VoxelStatistics::selectStd(logOddsUpdatedStats);

    std::vector<double> absErrorUpdatedBelief = errorUpdatedBelief;
    std::vector<double> absErrorUpdatedLogOdds = errorUpdatedLogOdds;
    for (unsigned int i = 0; i < absErrorUpdatedBelief.size(); ++i)
    {
        absErrorUpdatedBelief[i] = std::abs(absErrorUpdatedBelief[i]);
        absErrorUpdatedLogOdds[i] = std::abs(absErrorUpdatedLogOdds[i]);
    }
    _msg.stdErrorCorrelationBelief.push_back(pcc(stdUpdatedBelief, absErrorUpdatedBelief));
    _msg.stdErrorCorrelationLogOdds.push_back(pcc(stdUpdatedLogOdds, absErrorUpdatedLogOdds));

#ifndef MANY_STEPS
    // append current errors to complete error vectors
    _msg.errorCompleteLogOdds.insert(_msg.errorCompleteLogOdds.end(),
                                     _msg.errorLogOdds.begin(),
                                     _msg.errorLogOdds.end());
    _msg.errorCompleteBelief.insert(_msg.errorCompleteBelief.end(),
                                    _msg.errorBelief.begin(),
                                    _msg.errorBelief.end());

    // append current errors of updated voxels to complete updated error vectors
    _msg.errorCompleteUpdatedLogOdds.insert(_msg.errorCompleteUpdatedLogOdds.end(),
                                     absErrorUpdatedLogOdds.begin(),
                                     absErrorUpdatedLogOdds.end());
    _msg.errorCompleteUpdatedBelief.insert(_msg.errorCompleteUpdatedBelief.end(),
                                    absErrorUpdatedBelief.begin(),
                                    absErrorUpdatedBelief.end());

    // append current std devs to complete std dev vectors
    _msg.stdCompleteLogOdds.insert(_msg.stdCompleteLogOdds.end(),
                                     _msg.stdLogOdds.begin(),
                                     _msg.stdLogOdds.end());
    _msg.stdCompleteBelief.insert(_msg.stdCompleteBelief.end(),
                                    _msg.stdBelief.begin(),
                                    _msg.stdBelief.end());

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
    _msg.ismIncrement = Parameters::invSensor_increment;
    _msg.ismRampSize = Parameters::invSensor_rampSize;
    _msg.ismTopSize = Parameters::invSensor_topSize;
    _msg.ismRampSlope = Parameters::invSensor_rampSlope;

#ifndef MANY_STEPS
    double evolutionLogOdds = 0;
    for (double error : _msg.errorLogOdds)
        evolutionLogOdds += std::abs(error);
    evolutionLogOdds /= _msg.errorLogOdds.size();
    _msg.errorEvolutionLogOdds.push_back(evolutionLogOdds);
    double evolutionBelief = 0;
    for (double error : _msg.errorBelief)
        evolutionBelief += std::abs(error);
    evolutionBelief /= _msg.errorBelief.size();
    _msg.errorEvolutionBelief.push_back(evolutionBelief);

    _msg.stdLogOdds = VoxelStatistics::selectStd(logOddsCompleteStats);
    _msg.stdBelief = VoxelStatistics::selectStd(beliefCompleteStats);

    // append current std devs to complete std dev vectors
    _msg.stdCompleteLogOdds.insert(_msg.stdCompleteLogOdds.end(),
                                     _msg.stdLogOdds.begin(),
                                     _msg.stdLogOdds.end());
    _msg.stdCompleteBelief.insert(_msg.stdCompleteBelief.end(),
                                    _msg.stdBelief.begin(),
                                    _msg.stdBelief.end());
#endif

#ifdef PLANNER_2D_TEST
    _msg.trajectoryVoxels = (unsigned int) robot.trajectory().splineVoxelKeys(_trueMap).size();
    //ROS_INFO("Trajectory voxels: %d", (int)_msg.trajectoryVoxels);
    for (auto &position: robot.trajectory().splineVoxelPositions(_trueMap))
    {
        auto beliefVoxel = beliefMap.query(position);
        _msg.trajectoryOccupanciesBelief.push_back(beliefMap.getVoxelMean(beliefVoxel));
        _msg.trajectoryStdDevsBelief.push_back(beliefMap.getVoxelStd(beliefVoxel));
        auto logOddsVoxel = logOddsMap.query(position);
        _msg.trajectoryOccupanciesLogOdds.push_back(logOddsMap.getVoxelMean(logOddsVoxel));
        _msg.trajectoryStdDevsLogOdds.push_back(logOddsMap.getVoxelStd(logOddsVoxel));
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

    _publisher.publish(_msg);

    // TODO wait some time to ensure data is plotted
    //ros::Rate publishing_rate(29);
    //publishing_rate.sleep();
}

void Statistics::saveToFile(std::string filename) const
{
    rosbag::Bag bag;
    bag.open(filename, rosbag::bagmode::Write);
    bag.write("stats", ros::Time::now(), _msg);
    bag.close();
    ROS_INFO_STREAM("Saved statistics ROS bag file to " << filename);
}

double Statistics::pcc(const std::vector<double> &xs, const std::vector<double> &ys)
{
    //ROS_INFO("Actually updated voxels: %i and %i", (int)xs.size(), (int)ys.size());
    assert(xs.size() == ys.size());
    if (xs.size() != ys.size())
        ROS_ERROR("Error during PCC computation: input arrays have different lengths.");
    unsigned int n = xs.size();

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
//    stdX = std::sqrt(stdX);
//    stdY = std::sqrt(stdY);

//    n = len(x)
//    mx = x.mean()
//    my = y.mean()
//    xm, ym = x-mx, y-my
//    r_num = np.add.reduce(xm * ym)
//    r_den = np.sqrt(ss(xm) * ss(ym))
//    r = r_num / r_den

//    double ssp = 0.; // standard score product for x and y
//    for (unsigned int i = 0; i < n; ++i)
//        ssp += (xs[i] - meanX)/stdX * (ys[i] - meanY)/stdY;
//    ssp /= n - 1;
    double ssp = 0.; // standard score product for x and y
    for (unsigned int i = 0; i < n; ++i)
        ssp += (xs[i]-meanX) * (ys[i]-meanY);
    //ssp -= n * meanX * meanY;
    ssp /= std::sqrt(stdX * stdY);
    return ssp;
}

double Statistics::avg_distance(const std::vector<double> &xs, const std::vector<double> &ys)
{
    assert(xs.size() == ys.size());
    if (xs.size() != ys.size())
        ROS_ERROR("XS != YS!!!");
    unsigned int n = xs.size();

    double meanDst;
    for (unsigned int i = 0; i < n; ++i)
    {
        meanDst += std::abs(xs[i] - ys[i]);
    }
    meanDst /= n;
    return meanDst;
}

void Statistics::reset()
{
    _msg.step = 0;
    _msg.maxStep = 0;
    _msg.voxels = 0;

    _msg.errorBelief.clear();
    _msg.errorLogOdds.clear();
    _msg.errorEvolutionBelief.clear();
    _msg.errorEvolutionLogOdds.clear();

    _msg.errorCompleteBelief.clear();
    _msg.errorCompleteLogOdds.clear();

    _msg.updatedVoxels.clear();

    _msg.errorCompleteUpdatedBelief.clear();
    _msg.errorCompleteUpdatedLogOdds.clear();

    _msg.stdBelief.clear();
    _msg.stdLogOdds.clear();

    _msg.stdCompleteBelief.clear();
    _msg.stdCompleteLogOdds.clear();

    _msg.stdCompleteUpdatedBelief.clear();
    _msg.stdCompleteUpdatedLogOdds.clear();

    _msg.stdErrorCorrelationBelief.clear();
    _msg.stdErrorCorrelationLogOdds.clear();

    _msg.trajectoryVoxels = 0;
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
