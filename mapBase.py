import numpy as np
import scipy as sp

from paramList import *
import matplotlib.pyplot as plt
from scipy.interpolate import interp2d
from scipy.interpolate import splprep, splev

class VoxelGeometry:
    size = Par.voxelSize
    baseVoxelVertices_x = [-Par.voxelSize/2, Par.voxelSize/2, Par.voxelSize/2, -Par.voxelSize/2]
    baseVoxelVertices_y = [-Par.voxelSize / 2, -Par.voxelSize / 2, Par.voxelSize / 2, Par.voxelSize / 2]
    def __init__(self, id_inp, center_inp):
        self.id = id_inp
        self.center = center_inp
        self.vertices_x = self.center[0] + np.array(VoxelGeometry.baseVoxelVertices_x)
        self.vertices_y = self.center[1] + np.array(VoxelGeometry.baseVoxelVertices_y)

    def contains(self, px, py):
        return (self.center[0]-Par.voxelSize/2 <= px <= self.center[0]+Par.voxelSize/2)\
               and (self.center[1] - Par.voxelSize / 2 <= py <= self.center[1] + Par.voxelSize / 2)


class Voxel(VoxelGeometry):
    def __init__(self, density_inp, id_inp, center_inp):
        VoxelGeometry.__init__(self, id_inp, center_inp)
        self.density = density_inp

    def draw(self):
        # plt.text(self.center[0], self.center[1], self.id)
        color_intensity = 1.-0.5*self.density
        rgb_val = [color_intensity] * 3
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
        self.start = [-self.grid.voxelSize * 1., self.grid.yMin + self.grid.voxelSize / 2.]
        self.goal = [self.grid.xMax - self.grid.voxelSize / 2., 0]
        self.obstacles = [
            {
                # top right
                "xMin": self.grid.xMax - 10 * self.grid.voxelSize,
                "xMax": self.grid.xMax,
                "yMin": self.grid.yMax - 9 * self.grid.voxelSize,
                "yMax": self.grid.yMax
            },
            {
                # bottom right
                "xMin": self.grid.xMax - 10 * self.grid.voxelSize,
                "xMax": self.grid.xMax,
                "yMin": self.grid.yMin,
                "yMax": self.grid.yMin + 9 * self.grid.voxelSize
            }
        ]
        self.paths = [
            {
                "vertices": [self.start, (self.start[0], self.goal[1]), self.goal],
                "k": 1,
                "color": "red"
            },
            {
                "vertices": [
                    self.start,
                    (self.start[0] - Par.voxelSize * 4., self.goal[1]),
                    (self.start[0] + Par.voxelSize * 3., self.goal[1]),
                    self.goal],
                "k": 2,
                "color": "blue"
            },
            {
                "vertices": [
                    self.start,
                    (self.start[0] - Par.voxelSize * 5., self.goal[1] + Par.voxelSize * 5),
                    (self.start[0] - Par.voxelSize * 7., self.goal[1]),
                    (self.start[0], self.goal[1]),
                    self.goal],
                "k": 2,
                "color": "green"
            }
        ]

        for path in self.paths:
            pts = np.array(map(lambda x: [x[0], x[1]], path["vertices"]))
            tck, u = splprep(pts.T, u=None, s=0.0, per=0, k=path["k"])
            u_new = np.linspace(u.min(), u.max(), 100)
            x_new, y_new = splev(u_new, tck, der=0)
            path["interpolated"] = zip(x_new, y_new)

    @staticmethod
    def draw_point(point, color, size=14):
        plt.plot(point[0], point[1], marker='o', color=color, markersize=size)
