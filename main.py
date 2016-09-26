from simulatorClass import *
from mapHybrid import *
from planner import *
from results import *
from mapLogOdds import *

import numpy as np
import matplotlib.pyplot as plt

print('start the program')
plt.close("all")

np.random.seed(Par.seedValue)

sim = Simulator()
sim.setRobot(Par.initialPoseGlobal)
sim.draw()

map = MapHybrid()
map.draw()

mapLogOdds = MapLogOdds()
mapLogOdds.draw()

robotSensorBelief = RobotSensor(Par.initialPoseGlobal[0:1+1],Par.initialPoseGlobal[2])
robotSensorBelief.draw(map.plot_h['axis'])

planner = Planner()
results = Results(sim.trueMap)

# main loop
for k in range(Par.numSteps+1):
    print(k)
    z = sim.getObservation()
    sim.robotSensor.draw(sim.trueMap.grid.plot_h['axis'])

    map.update(z, robotSensorBelief.sensor)
    map.draw()
    # robotSensorBelief.draw(map.plot_h['axis'])

    mapLogOdds.update(z, robotSensorBelief.sensor)
    mapLogOdds.draw()
    # robotSensorBelief.robot.robotNeedRefresh = True
    # robotSensorBelief.sensor.sensorNeedRefresh = True
    # robotSensorBelief.draw(mapLogOdds.plot_h['axis'])

    # update robot pose
    (action, execution_time) = planner.pureRotation()
    sim.robotSensor.move(action, execution_time)
    robotSensorBelief.move(action, execution_time)

    if k == Par.numSteps:
        saveFlag = False
    else:
        saveFlag = False

    results.draw(map, mapLogOdds, k, saveFlag)

plt.show()
print "end of main"
