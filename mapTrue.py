import numpy as np
from mapBase import *

class TrueMap(BaseMap):
    def __init__(self):
        BaseMap.__init__(self)

        def in_rect(center):
            for rect in self.obstacles:
                if rect["xMin"] <= center[0] <= rect["xMax"] and rect["yMin"] <= center[1] <= rect["yMax"]:
                    return True
            return False

        self.voxels = []
        for voxGeom in self.grid.voxelsGeom:
            currentVox = Voxel(int(in_rect(voxGeom.center)), voxGeom.id, voxGeom.center)
            self.voxels.append(currentVox)

    def draw(self):
        plt.figure("TrueMap")
        for vox in self.voxels:
            vox.draw()

        BaseMap.draw_point(self.start, "#0066FF")
        BaseMap.draw_point(self.goal, "#FF6600")

        for p in self.paths:
            x, y = zip(*p["interpolated"])
            plt.plot(x, y, 'o-', color=p["color"])
