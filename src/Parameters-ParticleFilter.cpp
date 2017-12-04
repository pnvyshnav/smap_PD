#include "../include/Parameters.h"

#define DIMENSIONS 3

Parameters::NumType Parameters::voxelSize = 1;

Parameters::NumType Parameters::xMin = 0;
Parameters::NumType Parameters::xMax = 10;
Parameters::NumType Parameters::yMin = 0;
Parameters::NumType Parameters::yMax = 1;
Parameters::NumType Parameters::zMin = 0;
Parameters::NumType Parameters::zMax = 1;

bool Parameters::sensorTruncatedGaussianNoise = false;
Parameters::NumType Parameters::sensorRange = 10;
Parameters::NumType Parameters::sensorNoiseStd = (const Parameters::NumType) (1. * voxelSize);

Parameters::NumType Parameters::freeRadiusAroundCenter = 0; // irrelevant
// IRRELEVANT:
double Parameters::FakeRobotAngularVelocity = 5. * M_PI / 180.;
unsigned int Parameters::StereoCameraHorizontalPixels = 1;
unsigned int Parameters::StereoCameraVerticalPixels = 1;
double Parameters::StereoCameraHorizontalFOV = 40. * M_PI / 180.;    // quadratic pixels
double Parameters::StereoCameraVerticalFOV = StereoCameraHorizontalFOV * StereoCameraVerticalPixels *1. / StereoCameraHorizontalPixels *1.;

