#pragma once

#include <gp/gp.h>

#include "Observation.hpp"
#include "BeliefVoxel.h"
#include "Observable.hpp"
#include "Parameters.hpp"
#include "StatisticsMap.hpp"

struct GaussianProcessMapParameters
{
    double parameter1 = Parameters::gpParameter1;
    double parameter2 = Parameters::gpParameter2;
    double parameter3 = Parameters::gpParameter3;

    /**
     * Critical property that defines the covariance kernel
     * used by the Gaussian Processes.
     * Check out libgp for a documentation of this definition.
     */
    std::string kernel = "CovSum ( CovMatern5iso, CovNoise)";
};

/**
 * Mapping based on Gaussian Processes.
 * Samples from range sensor measurements with -1 for unoccupied
 * and 1 for occupied voxels.
 * The resulting estimated mean is normalized to [0,1].
 * The implementation is based on libgp and supports 2 or 3 dimensions,
 * defined by preprocessor declaration, e.g. #define DIMENSIONS 3
 * (DIMENSIONS is declared in Parameters.hpp).
 */
class GaussianProcessMap : public Observable
{
public:
    GaussianProcessMap();
    virtual ~GaussianProcessMap();

    /**
     * Updates GP map using the provided sensor range
     * measurements.
     * @param observation Sensor range measurements.
     * @return Always true.
     */
    bool update(const Observation &observation);

    /**
     * Computes mean and variance at the given position.
     * @param position Query position.
     * @return Belief at this position.
     */
    Belief belief(const octomap::point3d& position);

    /**
     * Computes voxel-based statistics for the whole grid.
     * @param trueMap The ground truth map.
     * @return Voxel statistics.
     */
    // TODO use smart pointers for QVoxel etc.
    std::vector<VoxelStatistics> stats(const TrueMap &trueMap);

    /**
     * Parameters for the Gaussian Processes.
     * Number of parameters is dependent on the kernel,
     * here assumed to be 3 parameters.
     */
    __attribute__((weak))
    static GaussianProcessMapParameters parameters;

private:
    libgp::GaussianProcess _gp;

    // for stats memory management
    std::vector<QPlainBeliefVoxel> _voxels;

    /**
     * Clears up voxels and stored Belief objects to
     * avoid memory leaks.
     */
    void _clearVoxels();
};

GaussianProcessMapParameters GaussianProcessMap::parameters;
