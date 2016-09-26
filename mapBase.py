import numpy as np

from paramList import *
import matplotlib.pyplot as plt

class VoxelGeometry:
    size = Par.voxelSize
    baseVoxelVertices_x = [-Par.voxelSize/2, Par.voxelSize/2, Par.voxelSize/2, -Par.voxelSize/2]
    baseVoxelVertices_y = [-Par.voxelSize / 2, -Par.voxelSize / 2, Par.voxelSize / 2, Par.voxelSize / 2]
    def __init__(self, id_inp, center_inp):
        self.id = id_inp
        self.center = center_inp
        self.vertices_x = self.center[0] + np.array(VoxelGeometry.baseVoxelVertices_x)
        self.vertices_y = self.center[1] + np.array(VoxelGeometry.baseVoxelVertices_y)


class Voxel(VoxelGeometry):
    def __init__(self, density_inp, id_inp, center_inp):
        VoxelGeometry.__init__(self, id_inp, center_inp)
        self.density = density_inp

    def draw(self):
        # plt.text(self.center[0], self.center[1], self.id)
        color_intensity = 1.-0.5*self.density
        rgb_val = [color_intensity, color_intensity, color_intensity]
        plt.fill(self.vertices_x, self.vertices_y, color = rgb_val)


#####################################################
#####################################################
class GridClass:
    def __init__(self):
        print('grid class constructor')
        self.xMin = Par.xMin
        self.xMax = Par.xMax
        self.yMin = Par.yMin
        self.yMax = Par.yMax
        self.voxelSize = Par.voxelSize

        self.xTicks = np.arange(self.xMin, self.xMax+0.1*self.voxelSize, self.voxelSize) # for inclusive arange
        self.yTicks = np.arange(self.yMin, self.yMax+0.1*self.voxelSize, self.voxelSize) # for inclusive arange

        self.xCenters = self.voxelSize/2. + self.xTicks[0:-1]
        self.yCenters = self.voxelSize/2. + self.yTicks[0:-1]

        self.numCells = (len(self.xCenters)) * (len(self.yCenters))

        self.voxelsGeom = []
        voxId = 0 # since we are using append down below, indexing has to start from 0, for later "accessing" commands
        for y in self.yCenters:
            for x in self.xCenters:
                voxel_xy = VoxelGeometry(voxId, [x,y])
                self.voxelsGeom.append(voxel_xy)
                voxId += 1

        self.plot_h = {'axis':None}

    def drawGrid(self):
        print('GridClass::drawGrid function')
        for x in self.xTicks:
            plt.plot([x,x],[self.yMin,self.yMax],'k')
        for y in self.yTicks:
            plt.plot([self.xMin,self.xMax],[y,y],'k')
        plt.xlim([self.xMin, self.xMax])
        plt.ylim([self.yMin, self.yMax])
        self.plot_h['axis'] = plt.gca()
        self.plot_h['axis'].set_aspect(1)



    def voxelsOnRayOrdered(self, ray_global):
        reso = Par.sensor_VoxelOnRayCheckingResolution
        ray_length = np.linalg.norm(ray_global['endPoint']-ray_global['startPoint'])
        assert np.abs(ray_length-Par.sensor_range)<1e-10
        num_check_pnts = ray_length/reso
        pnts_x = np.linspace((ray_global['startPoint'])[0], (ray_global['endPoint'])[0], num_check_pnts, endpoint=True)
        pnts_y = np.linspace((ray_global['startPoint'])[1], (ray_global['endPoint'])[1], num_check_pnts, endpoint=True)
        voxIndices = []
        voxCand = self.getVoxel(pnts_x[0], pnts_y[0])
        if voxCand is not None:
            voxIndices.append(voxCand)
        for x, y in zip(pnts_x, pnts_y):
            voxCand = self.getVoxel(x,y)
            if voxCand is not None and voxCand != voxIndices[-1]:
                voxIndices.append(voxCand)
        return voxIndices

    def getVoxel(self, x, y):
        inBounds = self.checkEnvBounds(x,y)
        if inBounds is False:
            return None
        # voxCenters = [vox.center for vox in self.voxelsGeom]
        idx = (np.abs(x - self.xCenters)).argmin()
        idy = (np.abs(y - self.yCenters)).argmin()
        id_linear = self.HighDim_ind_to_liner_ind(idx,idy)
        return id_linear

    def HighDim_ind_to_liner_ind(self,idx,idy):
        return idy*len(self.xCenters)+idx # This is true in python where all indices start from 0

    def checkEnvBounds(self,x,y):
        inBounds = (x<=self.xMax) and (x>=self.xMin) and (y<=self.yMax) and (y>=self.yMin)
        return inBounds


class BaseMap:
    def __init__(self):
        self.grid = GridClass()
        self.grid.drawGrid()


