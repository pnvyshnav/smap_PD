import numpy as np

class Par:

    # main simulation loop
    seedValue = 100
    numSteps = 17
    initialPoseGlobal = np.array([0.,0.,np.pi/2.])
    liveDrawing = 1

    # true grid
    voxelSize = .1
    xMin = -1. - voxelSize/2.
    xMax = 1. + voxelSize/2.
    yMin = -1. - voxelSize/2.
    yMax = 1. + voxelSize/2.

    # belief map
    numParticles = 51.
    priorMean = .5

    # sensor
    sensor_range = 1.  # meter
    sensor_ccdSize = .005  # meter
    sensor_numPixels = 15
    sensor_focalLength = .01  # meter
    sensor_fov = 2. * np.arctan((sensor_ccdSize / 2) / sensor_focalLength)
    sensor_VoxelOnRayCheckingResolution = voxelSize / 10.
    sensor_noise_std = sensor_range / 3.
    sensor_ProbSpuriousMeasurement = 0.  # This cannot be greater than 0 for now
    sensor_measurementFrequency = 10
    sensor_truncatedGaussianNoise = False

    # robot
    motionODE_integrationResolution = (1. / sensor_measurementFrequency) / 10

    # planner
    pureRotationAngularVelocity = -180.*np.pi/180.

    # results
    varianceMagnification = 2
    video = 0
    fullFileAddress = None
    largeErrorThreshold = .6
    lineWidth = 2

