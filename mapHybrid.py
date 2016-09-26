import numpy as np
from mapBeliefBase import *


class VoxelBeliefParicleBased(VoxelBelief):
    num_particles = Par.numParticles
    particle_supports = np.linspace(0, 1, Par.numParticles, endpoint=True)
    def __init__(self, id_inp, center_inp, pdf_inp):
        VoxelBelief.__init__(self, id_inp, center_inp)
        self.pdf = pdf_inp

    def setBelief(self, pdf_inp):
        self.pdf = pdf_inp
        self.needRefresh = 1

    def meanDensity(self):
        return np.sum(VoxelBeliefParicleBased.particle_supports * self.pdf)

    def varDensity(self):
        E_X2 = np.sum((VoxelBeliefParicleBased.particle_supports**2) * self.pdf)
        return E_X2 - (self.meanDensity()**2)

    def is_beliefValid(self):
        pdf_area = self.pdf.sum()
        if abs(pdf_area-1)<1e-10 and all(self.pdf>=0):
            return True
        else:
            return False

    def updatePDF(self, a, b):
        x = VoxelBeliefParicleBased.particle_supports
        affineLikelihoodFunction = a*x+b
        unNormalizedPosterior = affineLikelihoodFunction*self.pdf
        self.pdf = unNormalizedPosterior/np.sum(unNormalizedPosterior)
        assert self.is_beliefValid()
        self.needRefresh = 1

###############################################################
###############################################################
class MapHybrid(BeliefMapBase):
    def __init__(self):
        BeliefMapBase.__init__(self)
        n = VoxelBeliefParicleBased.num_particles
        coeff1 = (4.*n-2.-6.*(n-1)*Par.priorMean)/(n*n+n)
        coeff2 = -coeff1+2./n
        initial_belief = coeff2-(coeff2-coeff1)*VoxelBeliefParicleBased.particle_supports
        self.voxels = []
        for voxGeom in self.grid.voxelsGeom:
            currentVox = VoxelBeliefParicleBased(voxGeom.id, voxGeom.center, initial_belief)
            self.voxels.append(currentVox)
        assert self.voxels[0].is_beliefValid() # check validity of the initial belief

    def update(self, measurement, sensorBelief):
        sensorRotationBel = SO2.rotationMatrix(sensorBelief.orientation)
        sensorPositionBel = sensorBelief.position2D
        for pixSensor, pixMeasurement in zip(sensorBelief.pixelSensors, measurement):
            outPutPair = pixSensor.InverseCauseModel(pixMeasurement, mapBelief=self, sensorPosition=sensorPositionBel, sensorRotation=sensorRotationBel)
            voxel_glob_ids = outPutPair[1]
            causeProbabilities = outPutPair[0]
            causeProbabilitiesOnRay    = causeProbabilities['onRay']
            causeProbabilitiesInfinity = causeProbabilities['infinity']
            for id_glob, id_loc in zip(voxel_glob_ids, range(len(voxel_glob_ids))):
                Pr_beforeVox = np.sum(causeProbabilitiesOnRay[0:id_loc])
                Pr_onVox = causeProbabilitiesOnRay[id_loc]
                Pr_afterVox = np.sum(causeProbabilitiesOnRay[id_loc+1:]) + causeProbabilitiesInfinity
                m = self.voxels[id_glob].meanDensity()
                a = (1-m)*Pr_onVox - m*Pr_afterVox
                b = m*(1-m)*(Par.sensor_ProbSpuriousMeasurement + Pr_beforeVox) + m*Pr_afterVox
                assert self.voxels[id_glob].is_beliefValid()
                self.voxels[id_glob].updatePDF(a,b)
                assert self.voxels[id_glob].is_beliefValid()

    def computeBouncingProbabilityOnRay(self, voxGlobal_ids):
        bounceProb = []
        for id_glob in voxGlobal_ids:
            bounceProb.append(self.voxels[id_glob].meanDensity())
        return bounceProb

    def computeReachingProbabilityOnRay(self, voxGlobal_ids, bouncingProbabilities):
        reachingProb = []
        reachingProb.append(1-Par.sensor_ProbSpuriousMeasurement)
        for id_loc in range(1,len(voxGlobal_ids)):  # we skip the first one, as it is defined above before the loop
            # reachingProb[id_loc] = reachingProb[id_loc-1]*(1-bouncingProbabilities[id_loc-1])
            reachingProb.append(reachingProb[id_loc-1]*(1-bouncingProbabilities[id_loc-1]))
        return reachingProb