#include "../include/Parameters.h"


unsigned int Parameters::numParticles = 101;

#if defined(FAKE_2D)
    float Parameters::voxelSize = 0.1; //0.015; //0.08; // was 0.05
    float Parameters::xMin = -8; //-1.1 + voxelSize * 0.5;
    float Parameters::xMax = 10; //1.1 + voxelSize * 0.5;
    float Parameters::yMin = 0; //-1.1 + voxelSize * 0.5;
    float Parameters::yMax = 10; //1.1 + voxelSize * 0.5;
    float Parameters::zMin = 0;//(const Parameters::NumType) (-1. - voxelSize / 2.);
    float Parameters::zMax = voxelSize; //(const Parameters::NumType) (1. + voxelSize / 2.);

    Parameters::NumType Parameters::freeRadiusAroundCenter = 4 * voxelSize;

    bool Parameters::sensorTruncatedGaussianNoise = false;
    Parameters::NumType Parameters::sensorRange = (const Parameters::NumType) 1.; //1.5;
    Parameters::NumType Parameters::sensorNoiseStd = (const Parameters::NumType) (0.25 * voxelSize); //(sensorRange / 20.);

    unsigned int Parameters::StereoCameraHorizontalPixels = 60; //600;
    double Parameters::StereoCameraHorizontalFOV = 360. * M_PI / 180.;

    double Parameters::FakeRobotAngularVelocity = 15. * M_PI / 180.;
//    unsigned int Parameters::StereoCameraHorizontalPixels = 8;
//    double Parameters::StereoCameraHorizontalFOV = 40. * M_PI / 180.;
#elif defined(FAKE_3D)
    float Parameters::voxelSize = 0.125;
    float Parameters::xMin = -2. + voxelSize * 0.5;
    float Parameters::xMax = 2. + voxelSize * 0.5;
    float Parameters::yMin = -2. + voxelSize * 0.5;
    float Parameters::yMax = 2. + voxelSize * 0.5;
    float Parameters::zMin = -2;//(const Parameters::NumType) (-1. - voxelSize / 2.);
    float Parameters::zMax = 2; //(const Parameters::NumType) (1. + voxelSize / 2.);

    Parameters::NumType Parameters::freeRadiusAroundCenter = 1;

    bool Parameters::sensorTruncatedGaussianNoise = false;
    Parameters::NumType Parameters::sensorRange = (const Parameters::NumType) 3;
    Parameters::NumType Parameters::sensorNoiseStd = (const Parameters::NumType) (0.2);

    double Parameters::FakeRobotAngularVelocity = 5. * M_PI / 180.;

    unsigned int Parameters::StereoCameraHorizontalPixels = 8;
    unsigned int Parameters::StereoCameraVerticalPixels = 6;
    double Parameters::StereoCameraHorizontalFOV = 40. * M_PI / 180.;
    // quadratic pixels
    double Parameters::StereoCameraVerticalFOV = StereoCameraHorizontalFOV * StereoCameraVerticalPixels *1. / StereoCameraHorizontalPixels *1.;
#elif defined(REAL_3D)
    Parameters::NumType Parameters::voxelSize = 0.0625; //0.125;

    Parameters::NumType Parameters::xMin = -5.;
    Parameters::NumType Parameters::xMax =  3.4;
    Parameters::NumType Parameters::yMin = -4.6;
    Parameters::NumType Parameters::yMax =  7.8;
    Parameters::NumType Parameters::zMin = -4.1;
    Parameters::NumType Parameters::zMax =  4.5;

    bool Parameters::sensorTruncatedGaussianNoise = false;
    Parameters::NumType Parameters::sensorRange = (const Parameters::NumType) 10;
    Parameters::NumType Parameters::sensorNoiseStd = (const Parameters::NumType) .8 * voxelSize; //0.1;

    Parameters::NumType Parameters::freeRadiusAroundCenter = 0; // irrelevant
    // IRRELEVANT:
    double Parameters::FakeRobotAngularVelocity = 5. * M_PI / 180.;
    unsigned int Parameters::StereoCameraHorizontalPixels = 8;
    unsigned int Parameters::StereoCameraVerticalPixels = 6;
    double Parameters::StereoCameraHorizontalFOV = 40. * M_PI / 180.;    // quadratic pixels
    double Parameters::StereoCameraVerticalFOV = StereoCameraHorizontalFOV * StereoCameraVerticalPixels *1. / StereoCameraHorizontalPixels *1.;
#elif defined(REAL_2D)
    Parameters::NumType Parameters::voxelSize = .125; //0.125;

    Parameters::NumType Parameters::xMin = -10;
    Parameters::NumType Parameters::xMax =  20;
    Parameters::NumType Parameters::yMin = -10;
    Parameters::NumType Parameters::yMax =  20;
    Parameters::NumType Parameters::zMin = 0;
    Parameters::NumType Parameters::zMax = voxelSize;

    bool Parameters::sensorTruncatedGaussianNoise = false;
    Parameters::NumType Parameters::sensorRange = (const Parameters::NumType) 10;
    Parameters::NumType Parameters::sensorNoiseStd = (const Parameters::NumType) 1.8 * voxelSize; //0.1;

    Parameters::NumType Parameters::freeRadiusAroundCenter = 0; // irrelevant
    // IRRELEVANT:
    double Parameters::FakeRobotAngularVelocity = 5. * M_PI / 180.;
    unsigned int Parameters::StereoCameraHorizontalPixels = 8;
    unsigned int Parameters::StereoCameraVerticalPixels = 6;
    double Parameters::StereoCameraHorizontalFOV = 40. * M_PI / 180.;    // quadratic pixels
    double Parameters::StereoCameraVerticalFOV = StereoCameraHorizontalFOV * StereoCameraVerticalPixels *1. / StereoCameraHorizontalPixels *1.;
#else
    Parameters::NumType Parameters::voxelSize = 0.125;

    Parameters::NumType Parameters::xMin = -5;
    Parameters::NumType Parameters::xMax = 5;
    Parameters::NumType Parameters::yMin = -5;
    Parameters::NumType Parameters::yMax = 5;
    Parameters::NumType Parameters::zMin = -2;
    Parameters::NumType Parameters::zMax = 4;

    Parameters::NumType Parameters::freeRadiusAroundCenter = 4 * voxelSize;

    bool Parameters::sensorTruncatedGaussianNoise = false;
    Parameters::NumType Parameters::sensorRange = (const Parameters::NumType) 9;
    Parameters::NumType Parameters::sensorNoiseStd = (const Parameters::NumType) (sensorRange / 200.); // TODO has great effect on confidence
#endif

#if defined(PLANNER_2D_TEST)
    unsigned int Parameters::StereoCameraHorizontalPixels = 1;
    double Parameters::StereoCameraHorizontalFOV = 30. * M_PI / 180.;
#endif
