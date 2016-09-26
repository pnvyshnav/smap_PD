import numpy as np
from mapBeliefBase import *

class VoxelBeliefBernoulli(VoxelBelief):
    def __init__(self, id_inp, center_inp, pdf_inp):
        VoxelBelief.__init__(self, id_inp, center_inp)
        self.pdf = pdf_inp
        self.logOddPrior = np.log(Par.invSensor_prior/(1-Par.invSensor_prior))

    def setBelief(self, pdf_inp):
        self.pdf = pdf_inp
        self.needRefresh = 1

    def meanDensity(self):
        return self.pdf

    def varDensity(self):
        return self.pdf*(1-self.pdf)

    def is_beliefValid(self):
        if np.isscalar(self.pdf) and self.pdf>=0 and self.pdf<=1:
            return True
        else:
            return False

    def updatePDF(self, invSensorProb):
        logOdds_current = np.log(self.pdf/(1-self.pdf))
        logOdds_invSensor = np.log(invSensorProb/(1-invSensorProb))
        logOdds_next = logOdds_current + logOdds_invSensor - self.logOddPrior
        self.pdf = 1. - 1./(1.+np.exp(logOdds_next))
        assert self.is_beliefValid()
        self.needRefresh = 1

###############################################################
###############################################################
class MapLogOdds(BeliefMapBase):
    def __init__(self):
        BeliefMapBase.__init__(self)
        self.voxels = []
        for voxGeom in self.grid.voxelsGeom:
            currentVox = VoxelBeliefBernoulli(voxGeom.id, voxGeom.center, Par.priorMean)
            self.voxels.append(currentVox)
        assert self.voxels[0].is_beliefValid() # check validity of the initial belief

    def update(self, measurement, sensorBelief):
        sensorRotationBel = SO2.rotationMatrix(sensorBelief.orientation)
        sensorPositionBel = sensorBelief.position2D
        for pixSensor, pixMeasurement in zip(sensorBelief.pixelSensors, measurement):
            (causeProbabilitiesOnRay, voxel_glob_ids) = pixSensor.InverseSensorModel(pixMeasurement, mapBelief=self, sensorPosition=sensorPositionBel, sensorRotation=sensorRotationBel)
            for id_glob, id_loc in zip(voxel_glob_ids, range(len(voxel_glob_ids))):
                self.voxels[id_glob].updatePDF(causeProbabilitiesOnRay[id_loc])
                assert self.voxels[id_glob].is_beliefValid()
