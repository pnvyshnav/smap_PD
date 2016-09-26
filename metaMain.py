from paramList import *
import os
import time

uncertaintyLevel = (Par.sensor_range/3., Par.sensor_range/5., Par.sensor_range/10., Par.sensor_range/20., Par.sensor_range/50., Par.sensor_range/100.)
#uncertaintyLevel = (Par.sensor_range/3, Par.sensor_range/10, Par.sensor_range/100)
invSensor_increment = (0.05, 0.2, 0.4)
invSensor_rampSize = (Par.voxelSize/2., Par.voxelSize, Par.voxelSize*3, Par.voxelSize*10)
invSensor_topSize = (Par.voxelSize/2., Par.voxelSize, Par.voxelSize*3)

seed = range(50)

timeDirectoryName = time.strftime("results/%Y-%m-%d-%H-%M-%S")
if not os.path.exists(timeDirectoryName):
    os.makedirs(timeDirectoryName)
else:
    raise NotImplementedError

# for uncLev in uncertaintyLevel:
#     Par.sensor_noise_std = uncLev
#     fullDirectoryName = timeDirectoryName + '/std'
#     os.makedirs(Par.fullDirectoryName)
#     Par.fullFileAddress = timeDirectoryName + '/std_' + str(round(uncLev,3))
#     execfile("main.py")

# fullDirectoryName = timeDirectoryName + '/inv'
# os.makedirs(fullDirectoryName)
# for incr in invSensor_increment:
#     for rampS in invSensor_rampSize:
#         for topS in invSensor_topSize:
#             Par.invSensor_occupied = Par.invSensor_prior + incr
#             Par.invSensor_free = Par.invSensor_prior - incr
#             Par.invSensor_rampSize = rampS
#             Par.invSensor_topSize = topS
#             Par.invSensor_rampSlope = (Par.invSensor_occupied - Par.invSensor_free) / rampS
#             Par.fullFileAddress = timeDirectoryName + '/incr_' + str(round(incr, 3)) \
#                                   + '_ramp_' + str(round(rampS, 3)) + '_top_' + str(round(topS, 3))
#             print Par.fullFileAddress
#             execfile("main.py")


fullDirectoryName = timeDirectoryName + '/MonteCarlo'
os.makedirs(fullDirectoryName)
for s in seed:
    Par.seedValue = s
    Par.fullFileAddress = fullDirectoryName + '/seed' + str(round(s, 3))
    print Par.fullFileAddress
    execfile("main.py")