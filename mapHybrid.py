import numpy as np
from mapBeliefBase import *
import matplotlib.pyplot as plt

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

        #plt.ion()
        #plt.figure(13)
        #plt.plot(self.pdf)
        #plt.show()
        assert self.is_beliefValid()
        self.needRefresh = 1

###############################################################
###############################################################
class MapHybrid(BeliefMapBase):
    def __init__(self):
        BeliefMapBase.__init__(self)
        n = VoxelBeliefParicleBased.num_particles

        def prior_belief(prior):
            coeff1 = (4.*n-2.-6.*(n-1)*prior)/(n*n+n)
            coeff2 = -coeff1+2./n
            return coeff2-(coeff2-coeff1)*VoxelBeliefParicleBased.particle_supports

        def min_rect_distance(center):
            def dist(rect):
                r = {
                    "xMin": rect["xMin"] + Par.voxelSize,
                    "xMax": rect["xMax"] - Par.voxelSize,
                    "yMin": rect["yMin"] + Par.voxelSize,
                    "yMax": rect["yMax"] - Par.voxelSize
                }
                if r["xMin"] <= center[0] <= r["xMax"]:
                    # inside
                    if r["yMin"] <= center[1] <= r["yMax"]:
                        return 0

                    if center[1] < r["yMin"]:
                        return abs(center[1] - r["yMin"])
                    if center[1] > r["yMax"]:
                        return abs(center[1] - r["yMax"])
                if r["yMin"] <= center[1] <= r["yMax"]:
                    if center[0] < r["xMin"]:
                        return abs(center[0] - r["xMin"])
                    if center[0] > r["xMax"]:
                        return abs(center[0] - r["xMax"])
                elif center[0] < r["xMin"] and center[1] < r["yMin"]:
                    return np.sqrt((center[0] - r["xMin"])**2 + (center[1] - r["yMin"])**2)
                elif center[0] > r["xMax"] and center[1] < r["yMin"]:
                    return np.sqrt((center[0] - r["xMax"])**2 + (center[1] - r["yMin"])**2)
                elif center[0] < r["xMin"] and center[1] > r["yMax"]:
                    return np.sqrt((center[0] - r["xMin"])**2 + (center[1] - r["yMax"])**2)
                elif center[0] > r["xMax"] and center[1] > r["yMax"]:
                    return np.sqrt((center[0] - r["xMax"])**2 + (center[1] - r["yMax"])**2)

            return min(map(dist, self.obstacles))

        self.voxels = []
        for voxGeom in self.grid.voxelsGeom:
            #prior = min_rect_distance(voxGeom.center) * 3.0
            #if prior >= 1:
            #    prior = 1 - 1E-8
            #elif prior <= 0:
            #    prior = 1E-8
            currentVox = VoxelBeliefParicleBased(voxGeom.id, voxGeom.center, prior_belief(.5))
            self.voxels.append(currentVox)
        #assert self.voxels[0].is_beliefValid() # check validity of the initial belief

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
