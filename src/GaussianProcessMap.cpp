#include "../include/GaussianProcessMap.h"

#include <ecl/time.hpp>
#include <unordered_map>

ecl::StopWatch stopWatchGP;

double GaussianProcessMap::StdDevScalingFactor = Parameters::priorStd / 0.0301974;

GaussianProcessMap::GaussianProcessMap()
    : _gp(DIMENSIONS, parameters.kernel), _stdScalingComputed(false)
{
    std::cout << "The " << parameters.kernel << " kernel expects " << _gp.covf().get_param_dim() << " parameters." << std::endl;
    updateParameters();
}

GaussianProcessMap::~GaussianProcessMap()
{
    _clearVoxels();
}

bool GaussianProcessMap::update(const Observation &observation)
{
    if (observation.measurements().size() > 150)
        std::cout << "Updating GP map with " << observation.measurements().size() << " measurements..." << std::endl;
    stopWatchGP.restart();
    for (auto measurement : observation.measurements())
    {
        // Every voxel size length, sample from measurement ray.
        // The ray's end point is the occupied sample.
        for (auto &rayPoint : measurement.sensor.discretized(measurement.value))
        {
            // avoid spurious/infinity measurements to confuse GPOM
            if (!TrueMap::insideMap(rayPoint.position))
                continue;

            double point[DIMENSIONS] = {};
            point[0] = rayPoint.position.x();
            point[1] = rayPoint.position.y();
#if DIMENSIONS > 2
            point[2] = rayPoint.position.z();
#endif
            _gp.add_pattern(point, rayPoint.occupied ? 1 : -2);

            if (!_stdScalingComputed)
                computeStdDevScalingFactor();
        }
    }
    std::cout << "Time to update GP map: " << stopWatchGP.elapsed() << std::endl;
    return true;
}

Belief GaussianProcessMap::belief(const octomap::point3d &position)
{
    double point[DIMENSIONS] = {};
    point[0] = position.x();
    point[1] = position.y();
#if DIMENSIONS > 2
    point[2] = position.z();
#endif
    return Belief((_gp.f(point) + 1.) / 2., _gp.var(point));
//    return Belief(_gp.f(point), _gp.var(point));
}

std::vector<VoxelStatistics> GaussianProcessMap::stats(const TrueMap &trueMap)
{
    _clearVoxels();
    std::vector<VoxelStatistics> stats;
    for (unsigned int x = 0; x < Parameters::voxelsPerDimensionX; ++x)
    {
        for (unsigned int y = 0; y < Parameters::voxelsPerDimensionY; ++y)
        {
            for (unsigned int z = 0; z < Parameters::voxelsPerDimensionZ; ++z)
            {
                auto _x = Parameters::xMin + x * Parameters::voxelSize;
                auto _y = Parameters::yMin + y * Parameters::voxelSize;
                auto _z = Parameters::zMin + z * Parameters::voxelSize;
                Parameters::Vec3Type pos(_x, _y, _z);
                Belief *gpBelief = new Belief(belief(pos));

                QTrueVoxel trueVoxel = trueMap.query(pos);
                double error = (trueMap.getVoxelMean(trueVoxel) - gpBelief->mean());

                QPlainBeliefVoxel beliefVoxel = QPlainBeliefVoxel::voxel(gpBelief, pos, TrueMap::coordToKey(pos));
                _voxels.push_back(beliefVoxel);
//                std::cout << "GP Raw Std: " << std::sqrt(gpBelief->variance()) << std::endl;
                stats.push_back(VoxelStatistics(error, std::sqrt(gpBelief->variance()) * StdDevScalingFactor, &_voxels.back()));
            }
        }
    }
    return stats;
}

void GaussianProcessMap::_clearVoxels()
{
    for (auto &voxel : _voxels)
        delete voxel.node();
    _voxels.clear();
}

void GaussianProcessMap::computeStdDevScalingFactor()
{
    // compute the mode of predicted std devs, then
    // the reciprocal of the mode is the normalizer
    std::unordered_map<unsigned int, unsigned int> stds;
    const unsigned int voxelIncrement = 2;
    constexpr double precision = 1e6;

    std::cout << "Computing GP std scaling factor..." << std::endl;

    double point[DIMENSIONS] = {};
    unsigned int stdkey = 0;
    for (unsigned int x = 0; x < Parameters::voxelsPerDimensionX; x += voxelIncrement)
    {
        for (unsigned int y = 0; y < Parameters::voxelsPerDimensionY; y += voxelIncrement)
        {
            for (unsigned int z = 0; z < Parameters::voxelsPerDimensionZ; z += voxelIncrement)
            {
                point[0] = Parameters::xMin + x * Parameters::voxelSize;
                point[1] = Parameters::yMin + y * Parameters::voxelSize;
#if DIMENSIONS > 2
                point[2] = Parameters::zMin + z * Parameters::voxelSize;
#endif
                double std = std::sqrt(_gp.var(point));
                stdkey = (unsigned int) std::round(std * precision);
//                std::cout << "stdkey: " << stdkey << std::endl;
                if (stds.find(stdkey) == stds.end())
                    stds[stdkey] = 0;
                stds[stdkey]++;
            }
        }
    }
    auto mode = stdkey;
    for (auto &entry : stds)
    {
        if (entry.second > stds[mode])
            mode = entry.first;
    }
    StdDevScalingFactor = (mode / precision);
    std::cout << "GP StdDevScalingFactor: " << Parameters::priorStd << "/" << StdDevScalingFactor << std::endl;
    StdDevScalingFactor = Parameters::priorStd / StdDevScalingFactor;
    _stdScalingComputed = true;
}

void GaussianProcessMap::reset()
{
    _gp.clear_sampleset();
}

void GaussianProcessMap::updateParameters()
{
    Eigen::VectorXd params(_gp.covf().get_param_dim());
    params << parameters.parameter1, parameters.parameter2, parameters.parameter3; //1.0, 0.5; // characteristic length scale, signal variance
    _gp.covf().set_loghyper(params);
    _stdScalingComputed = false;
}
