#pragma once

#include <cmath>
//#include <eigen3/Eigen/Core>
#include <octomap/OcTree.h>

class Parameters
{
public:

    // TODO make configurable via ROS config

    typedef float NumType;
    typedef octomap::point3d Vec3Type;


    static constexpr NumType equalityThreshold = 1e-16;

    static const unsigned int numSteps = 100;
    static const unsigned int numParticles = 101;

    static constexpr NumType voxelSize = .1;

    static constexpr NumType xMin = -1. - voxelSize/2.;
    static constexpr NumType xMax = 1. + voxelSize/2.;
    static constexpr NumType yMin = -1. - voxelSize/2.;
    static constexpr NumType yMax = 1. + voxelSize/2.;
    static constexpr NumType zMin = -1. - voxelSize/2.;
    static constexpr NumType zMax = 1. + voxelSize/2.;

    static constexpr NumType xCenter = (xMax + xMin) / 2.0;
    static constexpr NumType yCenter = (yMax + yMin) / 2.0;
    static constexpr NumType zCenter = (zMax + zMin) / 2.0;
    static constexpr unsigned int maxDepth = (int)std::ceil((double)std::log2((xMax - xMin) / voxelSize));
    static constexpr unsigned int voxelsPerDimension = (xMax - xMin) / voxelSize;
    static constexpr NumType freeRadiusAroundCenter = .8;

    static const bool sensorTruncatedGaussianNoise = false;
    static constexpr NumType sensorRange = 1.;
    static constexpr NumType sensorNoiseStd = sensorRange / 20.;

    static constexpr NumType spuriousMeasurementProbability = 0; // cannot be > 0 for now

    static constexpr NumType priorMean = .5;

    /**
     * Sensor measurements without noise (deterministic case)
     */
    static const bool deterministicSensorMeasurements = true;
};
