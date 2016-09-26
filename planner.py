from paramList import *

class Action:
    def __init__(self, velocity, omega):
        self.velocity = velocity
        self.omega = omega

class Planner:

    def pureRotation(self):
        velocity = 0
        omega = Par.pureRotationAngularVelocity
        action = Action(velocity,omega)
        executionTime = 1./Par.sensor_measurementFrequency
        return (action, executionTime)