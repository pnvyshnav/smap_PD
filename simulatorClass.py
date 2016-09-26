from mapTrue import *
from robotAndSensor import *

import matplotlib.pyplot as plt


class Simulator:

    def __init__(self):
        print('simulator class constructor')
        self.trueMap = TrueMap()
        self.robotSensor = RobotSensor()

    def setRobot(self, robot_inp):
        self.robotSensor.set(robot_inp[0:1+1],robot_inp[2])

    def draw(self):
        axis = plt.gca()
        self.trueMap.draw()
        axis.set_aspect(1)
        self.robotSensor.draw(axis)
        if Par.liveDrawing:
            plt.pause(1e-6)

    def getObservation(self):
        return self.robotSensor.getObservation(self.trueMap)

