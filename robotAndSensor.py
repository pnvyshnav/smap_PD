from paramList import *
from SO2 import *
from truncatedGaussian import *

import matplotlib.pyplot as plt


class PlanarRobot:
    def __init__(self, position2D_inp, orientattion_inp):
        self.position2D = position2D_inp
        self.orientation = orientattion_inp
        self.plot_h = {'robot':None}
        self.robotNeedRefresh = 1

    def set(self, position2D_inp, orientattion_inp):
        self.position2D = position2D_inp
        self.orientation = orientattion_inp
        self.robotNeedRefresh = 1

    def move(self, action, duration):
        V = action.velocity
        omega = action.omega
        x = self.position2D[0]
        y = self.position2D[1]
        theta = self.orientation

        dt = Par.motionODE_integrationResolution
        for t in np.arange(0., duration+dt, dt):
            x = x + V*dt*np.cos(theta)
            y = y + V*dt*np.sin(theta)
            theta = theta + omega*dt
        self.position2D[0] = x
        self.position2D[1] = y
        self.orientation = theta
        self.robotNeedRefresh = 1

    def draw(self, axis):
        if self.robotNeedRefresh:
            position_xy = self.position2D
            if position_xy is None:
                print 'set the robot first'
            plt.sca(axis)
            if self.plot_h['robot'] is None:
                line2Dlist = plt.plot(position_xy[0],position_xy[1],'o',markersize=14)
                self.plot_h['robot'] = line2Dlist[0]
            else:
                self.plot_h['robot'].set_data(position_xy[0],position_xy[1])
            self.robotNeedRefresh = 0


class PixelSensor:
    def __init__(self, angle_inp):
        self.angle = angle_inp

    def getObservation(self, translation_arr, rotation, map_inp_const):
        ray_global = self.getRayGlobal(translation_arr, rotation)
        voxel_IDs = map_inp_const.grid.voxelsOnRayOrdered(ray_global)
        for vox_id in voxel_IDs:
            voxel = map_inp_const.voxels[vox_id]
            vox_density = voxel.density
            r = np.random.uniform()
            if r<vox_density: # voxel caused the observation
                causeVoxel = voxel
                return self.getObservationGivenCause(translation_arr, causeVoxel, noise=True)
        return 'hole'

    def getObservationGivenCause(self, position, causeVoxel, noise = True):
        deterministicRange = np.linalg.norm(causeVoxel.center - position)
        if noise is True:
            tg = TruncatedGaussian(deterministicRange,Par.sensor_noise_std,0.,Par.sensor_range)
            stochasticRange = tg.sample()
            return stochasticRange
        else:
            return deterministicRange

    def likelihoodGivenCause(self, measurement, position, cause_geom):
        if type(cause_geom) is not str:
            if measurement == 'hole':
                return 0.
            else:
                z_mostLikely = self.getObservationGivenCause(position, cause_geom, noise=False)
                tg = TruncatedGaussian(mean_inp=z_mostLikely, std_inp=Par.sensor_noise_std, lb_inp=0., ub_inp=Par.sensor_range)
                return tg.pdfValue(measurement)
        elif cause_geom == 'hole':
            likelihood = (measurement=='hole')
            return likelihood
        elif cause_geom == 'spurious':
            if measurement=='hole':
                return 0.
            else:
                return 1./Par.sensor_range  # uniform distribution


    def getRayGlobal(self, translation_arr, rotation):
        ray_global = {}
        ray_global['startPoint'] = translation_arr
        rayEnd_local = Par.sensor_range*np.array([np.cos(self.angle), np.sin(self.angle)])
        dim = translation_arr.size
        rayEnd_global = translation_arr.reshape(dim,1) + rotation*rayEnd_local.reshape(dim,1)
        ray_global['endPoint'] = np.array(rayEnd_global.T)[0] # This is correct, but very very error-prone line
        return ray_global

    def InverseCauseModel(self, measurement, mapBelief, sensorPosition, sensorRotation):
        rayGlobal = self.getRayGlobal(sensorPosition, sensorRotation)
        cause_global_ids = mapBelief.grid.voxelsOnRayOrdered(rayGlobal)
        bouncingProbabilities = mapBelief.computeBouncingProbabilityOnRay(cause_global_ids)
        reachingProbabilities = mapBelief.computeReachingProbabilityOnRay(cause_global_ids, bouncingProbabilities)
        causeProbabilities_prior = np.array(reachingProbabilities)*np.array(bouncingProbabilities)
        cause_voxels_geometry = [mapBelief.grid.voxelsGeom[i] for i in cause_global_ids]
        causeProb_posterior_onRay = []
        for cause_geom, prior in zip(cause_voxels_geometry, causeProbabilities_prior):
            likelihood = self.likelihoodGivenCause(measurement, sensorPosition, cause_geom)
            causeProb_posterior_onRay.append(likelihood*prior)
        causeProbabilities_posterior = {'onRay': causeProb_posterior_onRay}

        bouncingProbFromInfinity = 1.
        ReachingProbFromInfinity = (1-bouncingProbabilities[-1])*reachingProbabilities[-1]
        causeProbFromInfinity_prior = ReachingProbFromInfinity*bouncingProbFromInfinity
        measurementLikelihoodGivenInfinity = self.likelihoodGivenCause(measurement,sensorPosition,'hole')
        causeProbabilities_posterior['infinity'] = measurementLikelihoodGivenInfinity*causeProbFromInfinity_prior

        assert np.abs(np.sum(causeProbabilities_prior)+causeProbFromInfinity_prior - 1) < 1e-10
        eta = np.sum(causeProbabilities_posterior['onRay']) + causeProbabilities_posterior['infinity']
        causeProbabilities_posterior['onRay'] = causeProbabilities_posterior['onRay']/eta
        causeProbabilities_posterior['infinity'] = causeProbabilities_posterior['infinity']/eta
        assert np.abs(np.sum(causeProbabilities_posterior['onRay']) + causeProbabilities_posterior['infinity'] - 1) < 1e-10
        return (causeProbabilities_posterior, cause_global_ids)

    def InverseSensorModel(self, measurement, mapBelief, sensorPosition, sensorRotation):
        rayGlobal = self.getRayGlobal(sensorPosition, sensorRotation)
        cause_global_ids = mapBelief.grid.voxelsOnRayOrdered(rayGlobal)
        cause_voxels_geometry = [mapBelief.grid.voxelsGeom[i] for i in cause_global_ids]

        causeProbabilities_onRay = []
        if type(measurement) is not str:
            for vox in cause_voxels_geometry:
                distance = np.linalg.norm(vox.center - sensorPosition)
                if distance < measurement - Par.invSensor_rampSize/2.:
                    causeProbabilities_onRay.append(Par.invSensor_free)
                elif distance < measurement + Par.invSensor_rampSize/2.:
                    point_x = measurement - Par.invSensor_rampSize/2.
                    point_y = Par.invSensor_free
                    causeProbabilities_onRay.append(point_y+Par.invSensor_rampSlope*(distance-point_x))
                elif distance < measurement + Par.invSensor_rampSize/2. + Par.invSensor_topSize:
                    causeProbabilities_onRay.append(Par.invSensor_occupied)
                else:
                    causeProbabilities_onRay.append(Par.invSensor_prior)
                assert causeProbabilities_onRay[-1] < 1
        elif measurement == 'hole':
            causeProbabilities_onRay = Par.invSensor_free*np.ones(len(cause_global_ids))
        else:
            raise ValueError('the only acceptable string is "hole"')

        return (causeProbabilities_onRay, cause_global_ids)

class PlanarStereo:
    robotSensorBaseCone_xy = np.matrix([
        [Par.sensor_range * np.cos(-Par.sensor_fov / 2.), 0., Par.sensor_range * np.cos(Par.sensor_fov / 2.)],
        [Par.sensor_range * np.sin(-Par.sensor_fov / 2.), 0., Par.sensor_range * np.sin(Par.sensor_fov / 2.)]
                                        ])  # static variable

    def __init__(self, position2D_inp, orientattion_inp):
        self.position2D = position2D_inp
        self.orientation = orientattion_inp
        self.plot_h = {'sensor': None}
        self.sensorNeedRefresh = 1
        pixelBordersOnImagePlane = np.linspace(-Par.sensor_ccdSize/2.,Par.sensor_ccdSize/2.,Par.sensor_numPixels, endpoint=True)
        self.pixelAngles = np.arctan2(pixelBordersOnImagePlane,Par.sensor_focalLength)
        self.pixelSensors = []
        for pa in self.pixelAngles:
            ps = PixelSensor(pa)
            self.pixelSensors.append(ps)

    def set(self, position2D_inp, orientation_inp):
        self.position2D = position2D_inp
        self.orientation = orientation_inp
        self.sensorNeedRefresh = 1

    def draw(self, axis):
        if self.sensorNeedRefresh:
            theta = self.orientation
            position_xy = self.position2D
            num_vertices = PlanarStereo.robotSensorBaseCone_xy.shape[1]  # number of columns
            rotation = np.matrix([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
            translationTiled = np.tile(position_xy.reshape(2, 1), (1, num_vertices))
            points_global = rotation * PlanarStereo.robotSensorBaseCone_xy + translationTiled
            plt.sca(axis)
            if self.plot_h['sensor'] is None:
                line2Dlist = plt.plot(np.array(points_global)[0, :], np.array(points_global)[1, :], linewidth = Par.lineWidth, color = 'r')
                self.plot_h['sensor'] = line2Dlist[0]
            else:
                self.plot_h['sensor'].set_data(np.array(points_global)[0, :], np.array(points_global)[1, :])
            self.sensorNeedRefresh = 0
            # look at matplotlib.patches.Arc for the last part

    def getObservation(self, map_inp_const):
        z = []
        rotation_mat = SO2.rotationMatrix(self.orientation)
        translation_arr = self.position2D
        for pix in self.pixelSensors:
            z_pix = pix.getObservation(translation_arr, rotation_mat, map_inp_const)
            z.append(z_pix)
        return z


class RobotSensor:
    def __init__(self, position2D_inp = None, orientattion_inp = None):
        self.robot = PlanarRobot(position2D_inp, orientattion_inp)
        self.sensor = PlanarStereo(position2D_inp, orientattion_inp)

    def set(self, position2D_inp, orientattion_inp):
        self.robot.set(position2D_inp, orientattion_inp)
        self.sensor.set(position2D_inp, orientattion_inp)

    def draw(self,axis):
        self.robot.draw(axis)
        self.sensor.draw(axis)

    def getObservation(self, map_inp):
        return self.sensor.getObservation(map_inp)

    def move(self, action, duration):
        self.robot.move(action, duration)
        self.sensor.set(self.robot.position2D, self.robot.orientation)