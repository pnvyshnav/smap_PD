import numpy as np


class SO2:

    @staticmethod
    def rotationMatrix(angle):
        return np.matrix([[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]])