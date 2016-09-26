import matplotlib as plt
import numpy as np

from mapBase import *
from robotAndSensor import *


# belief Voxel
class VoxelBelief(VoxelGeometry):
    def __init__(self, id_inp, center_inp):
        VoxelGeometry.__init__(self, id_inp, center_inp)
        self.plot_h = {'meanPatch': None}
        self.needRefresh = 1

    def draw(self):
        if self.needRefresh:
            color_intensity = 1.-0.5*self.meanDensity()
            rgb_val = [color_intensity, color_intensity, color_intensity]
            if self.plot_h['meanPatch'] is None:
                polygonList = plt.fill(self.vertices_x, self.vertices_y, color = rgb_val)
                self.plot_h['meanPatch'] = polygonList[0]
            else:
                self.plot_h['meanPatch'].set_color(rgb_val)
            self.needRefresh = 0

    def meanDensity(self):
        raise NotImplementedError("Must override meanDensity in the child class")

class BeliefMapBase(BaseMap):
    def __init__(self):
        BaseMap.__init__(self)
        self.plot_h = {'axis': None}
        self.voxels = []
        for voxGeom in self.grid.voxelsGeom:
            currentVox = VoxelBelief(voxGeom.id, voxGeom.center) # -1 is because id's start from 1
            self.voxels.append(currentVox)

    def draw(self):
        if self.plot_h['axis'] is None:
            #self.plot_h['fig'] = plt.figure()
            plt.figure()
            self.grid.drawGrid()
            self.plot_h['axis'] = plt.gca()
            self.plot_h['axis'].set_aspect(1)
        else:
            plt.sca(self.plot_h['axis'])
        for vox in self.voxels:
            vox.draw()
        if Par.liveDrawing:
            plt.pause(1e-6)
