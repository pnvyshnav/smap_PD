#include "../include/Parameters.h"

Parameters::NumType Parameters::voxelSize = 1;

Parameters::NumType Parameters::xMin = 0;
Parameters::NumType Parameters::xMax = 30;
Parameters::NumType Parameters::yMin = 0;
Parameters::NumType Parameters::yMax = 3;
Parameters::NumType Parameters::zMin = 0;
Parameters::NumType Parameters::zMax = 1;

bool Parameters::sensorTruncatedGaussianNoise = false;
Parameters::NumType Parameters::sensorRange = 22;
Parameters::NumType Parameters::sensorNoiseStd = (const Parameters::NumType) .8 * voxelSize; //0.1;

Parameters::NumType Parameters::freeRadiusAroundCenter = 0; // irrelevant
// IRRELEVANT:
double Parameters::FakeRobotAngularVelocity = 5. * M_PI / 180.;
unsigned int Parameters::StereoCameraHorizontalPixels = 8;
unsigned int Parameters::StereoCameraVerticalPixels = 6;
double Parameters::StereoCameraHorizontalFOV = 40. * M_PI / 180.;    // quadratic pixels
double Parameters::StereoCameraVerticalFOV = StereoCameraHorizontalFOV * StereoCameraVerticalPixels *1. / StereoCameraHorizontalPixels *1.;

