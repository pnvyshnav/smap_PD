import numpy as np
from mapBase import *

class TrueMap(BaseMap):
    def __init__(self):
        BaseMap.__init__(self)
        trueDensity = np.round(np.random.uniform(0,1,self.grid.numCells))
        self.voxels = []
        for voxGeom in self.grid.voxelsGeom:
            if voxGeom.center[0]>4*voxGeom.size or voxGeom.center[0]<-4*voxGeom.size or voxGeom.center[1]>4*voxGeom.size or voxGeom.center[1]<-4*voxGeom.size:
                currentVox = Voxel(trueDensity[voxGeom.id], voxGeom.id, voxGeom.center) # note that voxel id's start from 0
            else:
                currentVox = Voxel(0., voxGeom.id, voxGeom.center)
            self.voxels.append(currentVox)

    def draw(self):
        for vox in self.voxels:
            vox.draw()