#pragma once

#include <cmath>
#include <eigen3/Eigen/Core>

class Parameters {
public:
    typedef float NumType;
    typedef Eigen::Vector3f Vec3Type;


    static constexpr NumType equalityThreshold = 1e-16;

    static const unsigned int numSteps = 100;
    static const unsigned int numParticles = 101;

    static constexpr NumType voxelSize = .1;

    static constexpr NumType xMin = -1. - voxelSize/2.;
    static constexpr NumType xMax = 1. + voxelSize/2.;
    static constexpr NumType yMin = -1. - voxelSize/2.;
    static constexpr NumType yMax = 1. + voxelSize/2.;
    static constexpr unsigned int maxDepth = (int)std::ceil((double)std::log2((xMax - xMin) / voxelSize));
    static constexpr unsigned int voxelsPerDimension = (xMax - xMin) / voxelSize;
};
