import matplotlib as plt
import numpy as np
from matplotlib.path import Path
import matplotlib.patches as patches

from mapBase import *
from robotAndSensor import *


# belief Voxel
class VoxelBelief(VoxelGeometry):
    def __init__(self, id_inp, center_inp):
        VoxelGeometry.__init__(self, id_inp, center_inp)
        self.plot_h = {'meanPatch': None}
        self.needRefresh = 1

    def draw(self, highlightColors=[]):
        if self.needRefresh:
            color_intensity = 1.-0.5*self.meanDensity()
            rgb_val = [color_intensity, color_intensity, color_intensity]
            if self.plot_h['meanPatch'] is None:
                polygonList = plt.fill(self.vertices_x, self.vertices_y, color = rgb_val)
                self.plot_h['meanPatch'] = polygonList[0]
            else:
                self.plot_h['meanPatch'].set_color(rgb_val)
            for highlightColor in highlightColors:
                plt.fill(self.vertices_x, self.vertices_y, color=highlightColor, alpha=0.3)
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
            plt.figure("BeliefMap")
            self.grid.drawGrid()
            self.plot_h['axis'] = plt.gca()
            self.plot_h['axis'].set_aspect(1)
        else:
            plt.sca(self.plot_h['axis'])

        for path in self.paths:
            path["voxels"] = set()

        for vox in self.voxels:
            highlight_colors = set()
            for path in self.paths:
                for p in path["interpolated"]:
                    if vox.contains(p[0], p[1]):
                        highlight_colors.add(path["color"])
                        path["voxels"].add(vox)
                        break
            vox.draw(list(highlight_colors))

        BaseMap.draw_point(self.start, "#0066FF")
        BaseMap.draw_point(self.goal, "#FF6600")

        for i, path in enumerate(self.paths):
            reachability = 1.
            for vox in path["voxels"]:
                reachability *= (1. - vox.meanDensity())
            print("path %i (%s) has a reachability of %.2f%%" % (i+1, path["color"], reachability*100.))

        for i, path in enumerate(self.paths):
            # hack: normalized sum of individual voxel variances (variance of sum of independent random variables)
            variance = sum(map(lambda vox: vox.varDensity(), path["voxels"])) / len(path["voxels"])
            #first = 1.
            #second = 1.
            #for vox in path["voxels"]:
            #    var = vox.varDensity()
            #    mean = vox.meanDensity()
            #    first *= var + mean ** 2.
            #    second *= mean ** 2.
            #variance = first - second
            print("path %i (%s) has a variance of %.6f" % (i+1, path["color"], variance))

        for p in self.paths:
            x, y = zip(*p["interpolated"])
            plt.plot(x, y, '-', color=p["color"])

        if Par.liveDrawing:
            plt.pause(1e-6)
