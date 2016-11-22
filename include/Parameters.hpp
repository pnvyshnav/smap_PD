#pragma once

#include <cmath>

#include <octomap/OcTree.h>

//#define FAKE_2D
#define FAKE_3D

class Parameters
{
public:

    // TODO make configurable via ROS config

    typedef double NumType;
    typedef octomap::point3d Vec3Type;

    static constexpr NumType equalityThreshold = 1e-16;

    static const unsigned int numParticles = 101;
    //
    // Fake sensor measurements
    //
    static const bool deterministicSensorMeasurements = false;
    static const unsigned int FakeRobotNumSteps = 500;

    //
    // Real sensor measurements
    //
    static constexpr float PointCloudResolution = 0.3f;

#if defined(FAKE_2D)
    static constexpr NumType voxelSize = 0.1;
    static constexpr NumType xMin = (const NumType) (-1. + voxelSize * 0.5);
    static constexpr NumType xMax = (const NumType) (1. + voxelSize * 0.5);
    static constexpr NumType yMin = (const NumType) (-1. + voxelSize * 0.5);
    static constexpr NumType yMax = (const NumType) (1. + voxelSize * 0.5);
    static constexpr NumType zMin = 0;//(const NumType) (-1. - voxelSize / 2.);
    static constexpr NumType zMax = voxelSize; //(const NumType) (1. + voxelSize / 2.);

    static constexpr NumType freeRadiusAroundCenter = 4 * voxelSize;

    static const bool sensorTruncatedGaussianNoise = false;
    static constexpr NumType sensorRange = (const NumType) 0.9;
    static constexpr NumType sensorNoiseStd = (const NumType) (sensorRange / 20.);

    static constexpr double FakeRobotAngularVelocity = 1. * M_PI / 180.;
#elif defined(FAKE_3D)
    static constexpr NumType voxelSize = 0.125;
    static constexpr NumType xMin = (const NumType) (-2. + voxelSize * 0.5);
    static constexpr NumType xMax = (const NumType) (2. + voxelSize * 0.5);
    static constexpr NumType yMin = (const NumType) (-2. + voxelSize * 0.5);
    static constexpr NumType yMax = (const NumType) (2. + voxelSize * 0.5);
    static constexpr NumType zMin = -2;//(const NumType) (-1. - voxelSize / 2.);
    static constexpr NumType zMax = 2; //(const NumType) (1. + voxelSize / 2.);

    static constexpr NumType freeRadiusAroundCenter = 1;

    static const bool sensorTruncatedGaussianNoise = false;
    static constexpr NumType sensorRange = (const NumType) 3;
    static constexpr NumType sensorNoiseStd = (const NumType) (sensorRange / 20.);

    static constexpr double FakeRobotAngularVelocity = 5. * M_PI / 180.;

    static const unsigned int StereoCameraHorizontalPixels = 8;
    static const unsigned int StereoCameraVerticalPixels = 6;
    static constexpr double StereoCameraHorizontalFOV = 40. * M_PI / 180.;
    // quadratic pixels
    static constexpr double StereoCameraVerticalFOV = StereoCameraHorizontalFOV * StereoCameraVerticalPixels *1. / StereoCameraHorizontalPixels *1.;
#else
    static constexpr NumType voxelSize = 0.125;

    static constexpr NumType xMin = -5;
    static constexpr NumType xMax = 5;
    static constexpr NumType yMin = -5;
    static constexpr NumType yMax = 5;
    static constexpr NumType zMin = -2;
    static constexpr NumType zMax = 4;

    static constexpr NumType freeRadiusAroundCenter = 4 * voxelSize;

    static const bool sensorTruncatedGaussianNoise = false;
    static constexpr NumType sensorRange = (const NumType) 9;
    static constexpr NumType sensorNoiseStd = (const NumType) (sensorRange / 200.); // TODO has great effect on confidence
#endif
    static constexpr NumType xCenter = (const NumType) ((xMax + xMin) / 2.0);
    static constexpr NumType yCenter = (const NumType) ((yMax + yMin) / 2.0);
    static constexpr NumType zCenter = (const NumType) ((zMax + zMin) / 2.0);

    static constexpr unsigned int voxelsPerDimensionX = (const unsigned int) ((xMax - xMin) / voxelSize);
    static constexpr unsigned int voxelsPerDimensionY = (const unsigned int) ((yMax - yMin) / voxelSize);
    static constexpr unsigned int voxelsPerDimensionZ = (const unsigned int) ((zMax - zMin) / voxelSize);

    static constexpr unsigned int voxelsTotal = voxelsPerDimensionX * voxelsPerDimensionY * voxelsPerDimensionZ;

    static constexpr NumType spuriousMeasurementProbability = 0; //TODO cannot be > 0 for now

    static constexpr NumType priorMean = .5; // TODO fix it

    //
    // Inverse Sensor Model
    //
    static constexpr NumType invSensor_prior = priorMean;
    static constexpr NumType invSensor_increment = 0.05;
    static constexpr NumType invSensor_occupied = /*invSensor_prior +*/ invSensor_increment;
    static constexpr NumType invSensor_free = /*invSensor_prior */- invSensor_increment;
    static constexpr NumType invSensor_rampSize = voxelSize;
    static constexpr NumType invSensor_topSize = voxelSize;
    static constexpr NumType invSensor_rampSlope = (invSensor_occupied - invSensor_free)/invSensor_rampSize;
};
