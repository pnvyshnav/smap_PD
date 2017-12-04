#pragma once

#include <cmath>
#include <unordered_set>

#include <pcl/common/common_headers.h>
#include <pcl/impl/point_types.hpp>

#include <octomap/OcTree.h>

#define DIMENSIONS 3
//#define FAKE_2D
//#define PLANNER_2D_TEST
//#define SIMULATE_TIME
//#define ONLY_HANDCRAFTED_TRAJECTORIES
//#define REPLANNING
//#define FAKE_3D
//#define REPEATED_RUNS
#define ISM_RUNS
//#define GP_RUNS
//#define MANY_STEPS
//#define COMPUTE_UPDATED_EVOLUTION
#define STATS_PARTICLES // store value of EVERY particle in the stats bag file

#define REAL_3D
//#define REAL_2D

#define INPUT_EUROC 1       // EuRoC MAV dataset
#define INPUT_SNAPDRAGON 2  // Snapdragon Flight

#define INPUT_TYPE INPUT_EUROC

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
    typedef pcl::PointXYZ PointType;

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
    static const unsigned int FakeRobotNumSteps = 600;

    //
    // Real sensor measurements
    //
    static constexpr float PointCloudResolution = 0.3f;

    //
    // Trajectory evaluation
    //
    static const bool EquidistantArcLengthSampling = true;
    static const unsigned int VelocityPlanningPoints = 250;
    static constexpr double SimulationTimeStep = 0.05;
    static constexpr double SimulationFinalTime = 5.0;


    static NumType voxelSize;

    static NumType xMin;
    static NumType xMax;
    static NumType yMin;
    static NumType yMax;
    static NumType zMin;
    static NumType zMax;

    static bool sensorTruncatedGaussianNoise;
    static NumType sensorRange;
    static NumType sensorNoiseStd;

    static NumType freeRadiusAroundCenter;

    static double FakeRobotAngularVelocity; // TODO IRRELEVANT

    static unsigned int StereoCameraHorizontalPixels;
    static unsigned int StereoCameraVerticalPixels;
    static double StereoCameraHorizontalFOV;
    static double StereoCameraVerticalFOV;

    static inline NumType xCenter() { return (const NumType) ((xMax + xMin) / 2.0); }
    static inline NumType yCenter() { return (const NumType) ((yMax + yMin) / 2.0); }
    static inline NumType zCenter() { return (const NumType) ((zMax + zMin) / 2.0); }
    static inline Vec3Type center()
    {
        return Vec3Type(xCenter(), yCenter(), zCenter());
    }

    static unsigned int voxelsPerDimensionX() { return (unsigned int) ((xMax - xMin) / voxelSize); }
    static unsigned int voxelsPerDimensionY() { return (unsigned int) ((yMax - yMin) / voxelSize); }
    static unsigned int voxelsPerDimensionZ() { return (unsigned int) ((zMax - zMin) / voxelSize); }

    static unsigned int voxelsTotal()
    {
        return voxelsPerDimensionX() * voxelsPerDimensionY() * voxelsPerDimensionZ();
    }

    static constexpr NumType spuriousMeasurementProbability = 0; //TODO cannot be > 0 for now

    static constexpr NumType priorMean = .5;
    static constexpr NumType priorStd = .5;

    //
    // Inverse Sensor Model
    //
    static constexpr NumType invSensor_prior = priorMean;
    static constexpr NumType invSensor_increment = 0.15; //0.15; // 0.05; //0.25; //0.01; // 0.25
    static constexpr NumType invSensor_occupied = invSensor_increment;
    static constexpr NumType invSensor_free = -invSensor_increment;
    static constexpr NumType invSensor_rampSize = 0.05; //0.05;
    static constexpr NumType invSensor_topSize = 0.05; //0.05;
    static constexpr NumType invSensor_rampSlope = (invSensor_occupied - invSensor_free)/invSensor_rampSize;

    //
    // Gaussian Processes Mapping
    //
    static constexpr double gpParameter1 = -2.5; //-2.5; //-2.5;
    static constexpr double gpParameter2 = -2.5; //-3.5; //-2.5;
    static constexpr double gpParameter3 = -2.5; //-0.5; //-2.5;
};
