#pragma once

#include <cmath>
#include <unordered_set>

#include <pcl/common/common_headers.h>
#include <pcl/impl/point_types.hpp>

#include <octomap/OcTree.h>

#define FAKE_2D
#define PLANNER_2D_TEST
//#define SIMULATE_TIME
//#define ONLY_HANDCRAFTED_TRAJECTORIES
#define REPLANNING
//#define FAKE_3D
//#define REPEATED_RUNS
//#define ISM_RUNS
#define MANY_STEPS
//#define COMPUTE_UPDATED_EVOLUTION

//#define REAL_3D

#define ENABLE_VISUALIZATION

//#define PUBLISH_STATS // publish statistics via ROS topic
//#define SLIM_STATS


class Parameters
{
public:

    // TODO make configurable via ROS config

    typedef double NumType;
    typedef octomap::point3d Vec3Type;
    typedef std::unordered_set<octomap::OcTreeKey, octomap::OcTreeKey::KeyHash> KeySet;
    typedef pcl::PointXYZI PointType;

    // Provides a hash function on octomap point3d
    struct PositionHash
    {
        size_t operator()(const octomap::point3d& position) const
        {
            int x = (int)(position.x() / Parameters::voxelSize);
            int y = (int)(position.y() / Parameters::voxelSize);
            int z = (int)(position.z() / Parameters::voxelSize);
            // a hashing function (same as octomap::OcTreeKey::KeyHash)
            return (size_t) (x + 1337 * y + 345637 * z);
        }
    };
    typedef std::unordered_set<octomap::point3d, PositionHash> PositionSet;

    static constexpr NumType equalityThreshold = 1e-8;

    static const unsigned int numParticles = 101;
    //
    // Fake sensor measurements
    //
    static const bool deterministicSensorMeasurements = false;
    static const unsigned int FakeRobotNumSteps = 30;

    //
    // BSplineTrajectory evaluation
    //
    static const bool EquidistantArcLengthSampling = true;
    static const unsigned int VelocityPlanningPoints = 250;
    static constexpr double SimulationTimeStep = 0.05;
    static constexpr double SimulationFinalTime = 5.0;

#if defined(FAKE_2D)
    static constexpr float voxelSize = 0.1;
    static constexpr float xMin = -1. + voxelSize * 0.5;
    static constexpr float xMax = 1. + voxelSize * 0.5;
    static constexpr float yMin = -1. + voxelSize * 0.5;
    static constexpr float yMax = 1. + voxelSize * 0.5;
    static constexpr float zMin = 0;//(const NumType) (-1. - voxelSize / 2.);
    static constexpr float zMax = voxelSize; //(const NumType) (1. + voxelSize / 2.);

    static constexpr NumType freeRadiusAroundCenter = 4 * voxelSize;

    static const bool sensorTruncatedGaussianNoise = false;
    static constexpr NumType sensorRange = (const NumType) 1.; // TODO was 1.5
    static constexpr NumType sensorNoiseStd = (const NumType) 0.1; // TODO was 0.2 //(sensorRange / 10.); // TODO revert

    static constexpr double FakeRobotAngularVelocity = 15. * M_PI / 180.;
//    static const unsigned int StereoCameraHorizontalPixels = 8;
//    static constexpr double StereoCameraHorizontalFOV = 40. * M_PI / 180.;
#elif defined(FAKE_3D)
    static constexpr float voxelSize = 0.125;
    static constexpr float xMin = -2. + voxelSize * 0.5;
    static constexpr float xMax = 2. + voxelSize * 0.5;
    static constexpr float yMin = -2. + voxelSize * 0.5;
    static constexpr float yMax = 2. + voxelSize * 0.5;
    static constexpr float zMin = -2;//(const NumType) (-1. - voxelSize / 2.);
    static constexpr float zMax = 2; //(const NumType) (1. + voxelSize / 2.);

    static constexpr NumType freeRadiusAroundCenter = 1;

    static const bool sensorTruncatedGaussianNoise = false;
    static constexpr NumType sensorRange = (const NumType) 3;
    static constexpr NumType sensorNoiseStd = (const NumType) (0.2);

    static constexpr double FakeRobotAngularVelocity = 5. * M_PI / 180.;

    static const unsigned int StereoCameraHorizontalPixels = 8;
    static const unsigned int StereoCameraVerticalPixels = 6;
    static constexpr double StereoCameraHorizontalFOV = 40. * M_PI / 180.;
    // quadratic pixels
    static constexpr double StereoCameraVerticalFOV = StereoCameraHorizontalFOV * StereoCameraVerticalPixels *1. / StereoCameraHorizontalPixels *1.;
#elif defined(REAL_3D)
    static constexpr NumType voxelSize = 0.0625; //0.125;

    static constexpr NumType xMin = -5.;
    static constexpr NumType xMax =  3.4;
    static constexpr NumType yMin = -4.6;
    static constexpr NumType yMax =  7.8;
    static constexpr NumType zMin = -0.1;
    static constexpr NumType zMax =  4.5;

    static const bool sensorTruncatedGaussianNoise = false;
    static constexpr NumType sensorRange = (const NumType) 10;
    static constexpr NumType sensorNoiseStd = (const NumType) 0.1;

    static constexpr NumType freeRadiusAroundCenter = 0; // irrelevant
    // IRRELEVANT:
    static constexpr double FakeRobotAngularVelocity = 5. * M_PI / 180.;
    static const unsigned int StereoCameraHorizontalPixels = 8;
    static const unsigned int StereoCameraVerticalPixels = 6;
    static constexpr double StereoCameraHorizontalFOV = 40. * M_PI / 180.;    // quadratic pixels
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

#if defined(PLANNER_2D_TEST)
    static const unsigned int StereoCameraHorizontalPixels = 16; // TODO was 8
    static constexpr double StereoCameraHorizontalFOV = 90. * M_PI / 180.;
#endif

    static constexpr NumType xCenter = (const NumType) ((xMax + xMin) / 2.0);
    static constexpr NumType yCenter = (const NumType) ((yMax + yMin) / 2.0);
    static constexpr NumType zCenter = (const NumType) ((zMax + zMin) / 2.0);

    static constexpr unsigned int voxelsPerDimensionX = (const unsigned int) ((xMax - xMin) / voxelSize);
    static constexpr unsigned int voxelsPerDimensionY = (const unsigned int) ((yMax - yMin) / voxelSize);
    static constexpr unsigned int voxelsPerDimensionZ = (const unsigned int) ((zMax - zMin) / voxelSize);

    static constexpr unsigned int voxelsTotal = voxelsPerDimensionX * voxelsPerDimensionY * voxelsPerDimensionZ;

    static constexpr NumType spuriousMeasurementProbability = 0; //TODO cannot be > 0 for now

    static constexpr NumType priorMean = .5;
    static constexpr NumType priorStd = .5;

    //
    // Real sensor measurements
    //
    static constexpr float PointCloudResolution = (float)voxelSize;

    //
    // Inverse Sensor Model
    //
    static constexpr NumType invSensor_prior = priorMean;
    static constexpr NumType invSensor_increment = 0.05;
    static constexpr NumType invSensor_occupied = invSensor_increment;
    static constexpr NumType invSensor_free = -invSensor_increment;
    static constexpr NumType invSensor_rampSize = 0.1;
    static constexpr NumType invSensor_topSize = 0.1;
    static constexpr NumType invSensor_rampSlope = (invSensor_occupied - invSensor_free)/invSensor_rampSize;
};
