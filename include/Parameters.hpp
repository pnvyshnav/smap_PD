#pragma once

#include <cmath>

#include <octomap/OcTree.h>

class Parameters
{
public:

    // TODO make configurable via ROS config

    typedef double NumType;
    typedef octomap::point3d Vec3Type;

    static constexpr NumType equalityThreshold = 1e-16;

    static const unsigned int numParticles = 101;

    static constexpr NumType voxelSize = .1;

    static constexpr NumType xMin = (const NumType) (-1. + voxelSize * 0.5);
    static constexpr NumType xMax = (const NumType) (1. + voxelSize * 0.5);
    static constexpr NumType yMin = (const NumType) (-1. + voxelSize * 0.5);
    static constexpr NumType yMax = (const NumType) (1. + voxelSize * 0.5);
    static constexpr NumType zMin = 0; //(const NumType) (-1. - voxelSize / 2.);
    static constexpr NumType zMax = voxelSize; //(const NumType) (1. + voxelSize / 2.);

    static constexpr NumType xCenter = (const NumType) ((xMax + xMin) / 2.0);
    static constexpr NumType yCenter = (const NumType) ((yMax + yMin) / 2.0);
    static constexpr NumType zCenter = (const NumType) ((zMax + zMin) / 2.0);
    static constexpr unsigned int voxelsPerDimensionX = (const unsigned int) ((xMax - xMin) / voxelSize);
    static constexpr unsigned int voxelsPerDimensionY = (const unsigned int) ((yMax - yMin) / voxelSize);
    static constexpr unsigned int voxelsPerDimensionZ = (const unsigned int) ((zMax - zMin) / voxelSize);
    static constexpr NumType freeRadiusAroundCenter = 4 * voxelSize;

    static const bool sensorTruncatedGaussianNoise = false;
    static constexpr NumType sensorRange = (const NumType) 0.9;
    static constexpr NumType sensorNoiseStd = (const NumType) (sensorRange / 20.);

    static constexpr NumType spuriousMeasurementProbability = 0; // cannot be > 0 for now

    static constexpr NumType priorMean = .5;

    /**
     * Sensor measurements without noise (deterministic case)
     */
    static const bool deterministicSensorMeasurements = false;
    static constexpr double FakeRobotAngularVelocity = 1. * M_PI / 180.;
    static const unsigned int FakeRobotNumSteps = 10000;
};
