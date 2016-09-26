from simulatorClass import *
from mapHybrid import *

from planner import *
from results import *

import numpy as np
import matplotlib.pyplot as plt

import sys

print('start the program')
plt.close("all")

np.random.seed(Par.seedValue)

sim = Simulator()
sim.setRobot(Par.initialPoseGlobal)
sim.draw()

map = MapHybrid()
map.draw()

trueMap = TrueMap()

planner = Planner()
results = Results(trueMap)

for pathId, path in enumerate(map.paths):
    print("\nEvaluating path %i (%s)" % (pathId+1, path["color"]))

    sim = Simulator()

    map = MapHybrid()
    map.draw()

    measurePositions = path["interpolated"][::30]
    totalSteps = len(measurePositions) * (Par.numSteps+1)
    stepCounter = 0.
    for mp in measurePositions:
        pose = np.array(list(mp) + [Par.initialPoseGlobal[2]])
        sim.setRobot(pose)
        sim.draw()
        robotSensorBelief = RobotSensor(pose[0:1+1], Par.initialPoseGlobal[2])
        robotSensorBelief.draw(map.plot_h['axis'])

        # main loop
        for k in range(Par.numSteps+1):
            stepCounter += 1

            sys.stdout.write("\r\tEvaluated %.2f%%\t\t" % (stepCounter*100./totalSteps))
            sys.stdout.flush()

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

        del robotSensorBelief

    sys.stdout.write("\r\t\t\t\t\t\t\t\t\t\n")
    sys.stdout.flush()
    map.evaluate_path(pathId)

plt.show()
print "end of main"
