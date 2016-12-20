#include "../include/TrueMap.h"
#include "../include/Statistics.h"

#include <smap/smapStats.h>
#include <rosbag/bag.h>



Statistics::Statistics(const TrueMap &trueMap) : _trueMap(trueMap)
{
    _nh = new ros::NodeHandle;
    _publisher = _nh->advertise<smap::smapStats>("stats", 1);
}

Statistics::~Statistics()
{
    delete _nh;
}

void Statistics::update(const LogOddsMap &logOddsMap, const BeliefMap &beliefMap, const FakeRobot<> &robot)
{
    _msg.step = robot.currentStep();
    _msg.maxStep = Parameters::FakeRobotNumSteps;

    _msg.errorLogOdds = logOddsMap.error(_trueMap);
    _msg.errorBelief = beliefMap.error(_trueMap);

    assert(_msg.errorLogOdds.size() == _msg.errorBelief.size());

    _msg.voxels = _msg.errorBelief.size();

    std::vector<double> errorUpdatedBelief = beliefMap.errorLastUpdated(_trueMap);
    std::vector<double> errorUpdatedLogOdds = logOddsMap.errorLastUpdated(_trueMap);
    assert(errorUpdatedBelief.size() == errorUpdatedLogOdds.size());
    _msg.updatedVoxels.push_back(beliefMap.lastUpdatedVoxels.size());

    assert(beliefMap.lastUpdatedVoxels.size() == logOddsMap.lastUpdatedVoxels.size());
    assert(beliefMap.lastUpdatedVoxels.size() == errorUpdatedBelief.size());

    std::vector<double> stdUpdatedBelief, stdUpdatedLogOdds;
    //ROS_INFO("Updated Voxels: %d", (int)beliefMap.lastUpdatedVoxels.size());
    for (auto &v : beliefMap.lastUpdatedVoxels)
    {
        if (v.type != GEOMETRY_VOXEL)
        {
            ROS_WARN("Skipped std belief");
            continue;
        }
        stdUpdatedBelief.push_back(std::sqrt(v.node()->getValue()->variance()));
    }
    for (auto &v : logOddsMap.lastUpdatedVoxels)
    {
        if (v.type != GEOMETRY_VOXEL)
        {
            ROS_WARN("Skipped std log odds");
            continue;
        }
        double p = v.node() == NULL ? Parameters::priorMean : v.node()->getOccupancy();
        // Bernoulli variance
        stdUpdatedLogOdds.push_back(std::sqrt(p * (1. - p)));
    }

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

    _msg.stdLogOdds.clear();
    for (auto &voxel : logOddsMap.voxels())
    {
        if (voxel.type != GEOMETRY_VOXEL)
            continue;
        double pdf = voxel.node() == NULL ? Parameters::priorMean : voxel.node()->getOccupancy();
        // Bernoulli variance
        _msg.stdLogOdds.push_back(std::sqrt(pdf * (1. - pdf)));
    }

    _msg.stdBelief.clear();
    for (auto &voxel : beliefMap.voxels())
    {
        if (voxel.type != GEOMETRY_VOXEL)
            continue;
        _msg.stdBelief.push_back(std::sqrt(voxel.node()->getValue()->variance()));
    }

    // append current std devs to complete std dev vectors
    _msg.stdCompleteLogOdds.insert(_msg.stdCompleteLogOdds.end(),
                                     _msg.stdLogOdds.begin(),
                                     _msg.stdLogOdds.end());
    _msg.stdCompleteBelief.insert(_msg.stdCompleteBelief.end(),
                                    _msg.stdBelief.begin(),
                                    _msg.stdBelief.end());
#endif

    _publisher.publish(_msg);

    // TODO wait some time to ensure the data gets plotted
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
        ROS_ERROR("XS != YS!!!");
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
