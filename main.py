from simulatorClass import *
from mapHybrid import *

from planner import *
from results import *

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

trueMap = TrueMap()


measurePositions = [Par.initialPoseGlobal]

planner = Planner()
results = Results(trueMap)

for mp in measurePositions:
    robotSensorBelief = RobotSensor(mp[0:1+1],mp[2])
    robotSensorBelief.draw(map.plot_h['axis'])

    # main loop
    for k in range(Par.numSteps+1):
        print(k)
        z = sim.getObservation()
        sim.robotSensor.draw(sim.trueMap.grid.plot_h['axis'])

        map.update(z, robotSensorBelief.sensor)
        map.draw()
        robotSensorBelief.draw(map.plot_h['axis'])
        robotSensorBelief.robot.robotNeedRefresh = True
        robotSensorBelief.sensor.sensorNeedRefresh = True

        # update robot pose
        (action, execution_time) = planner.pureRotation()
        sim.robotSensor.move(action, execution_time)
        robotSensorBelief.move(action, execution_time)

        if k == Par.numSteps:
            saveFlag = False
        else:
            saveFlag = False

        results.draw(map, None, k, saveFlag)

plt.show()
print "end of main"
