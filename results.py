import numpy as np
import matplotlib.pyplot as plt
import os
from paramList import *

class Results:
    def __init__(self, mapTrue):
        self.videoframe = '/tmp/videoFrame%03d.png'
        self.frameNumber = 0
        fh = plt.figure()
        self.fig = fh
        self.fig.suptitle('step = %d' % self.frameNumber)
        self.trueDensities = np.array([vox.density for vox in mapTrue.voxels])
        self.axisHybrid = fh.add_subplot(211)
        self.fig_fullError = plt.figure('FullError')
        self.axis_fullError = self.fig_fullError.add_subplot(111)
        self.fig_zoomedInconsistency = plt.figure('zoomedOnInconsistencies')
        self.axisHybridLargeErrors = self.fig_zoomedInconsistency.add_subplot(211)

        self.mag = Par.varianceMagnification

        self.errorHybridCurve = None

        self.ErrorOverFullMapHybrid = []
        self.ErrorOverFullMapLogOdds = []

        # window = plt.get_current_fig_manager().window
        # w = window.winfo_screenwidth()
        # h = window.winfo_screenheight()
        # window.wm_geomtery('=%dx%d' % (w, h))
        # plt.subplots_adjust(left=0.05, right=0.95, bottom=0.05, top=0.95, wspace=0, hspace=0)

    def draw(self, mapHybrid, mapLogOdds, timeStep, saveLogFlag):
        meanVectorHybrid = np.array([vox.meanDensity() for vox in mapHybrid.voxels])

        stdVectorHybrid = np.array([np.sqrt(vox.varDensity()) for vox in mapHybrid.voxels])

        errorHybrid = meanVectorHybrid - self.trueDensities

        self.ErrorOverFullMapHybrid.append(np.sum(np.abs(errorHybrid)))

        thresh = Par.largeErrorThreshold
        ind_large = np.nonzero((abs(errorHybrid)>thresh))
        largeErrorsHybrid = errorHybrid[ind_large]
        largeErrVarianceHybrid = stdVectorHybrid[ind_large]

        if saveLogFlag:
            logOutput = (meanVectorHybrid, self.trueDensities,
                         stdVectorHybrid,
                         self.ErrorOverFullMapHybrid, self.ErrorOverFullMapLogOdds)
            np.save(Par.fullFileAddress, logOutput)

        if Par.liveDrawing:
            plt.sca(self.axisHybrid)
            plt.cla()
            plt.plot(errorHybrid)
            plt.plot(-stdVectorHybrid*self.mag, 'r')
            plt.plot(stdVectorHybrid*self.mag, 'r')

            plt.sca(self.axisHybridLargeErrors)
            plt.cla()
            plt.plot(largeErrorsHybrid)
            plt.plot(-largeErrVarianceHybrid*self.mag,'r')
            plt.plot(largeErrVarianceHybrid*self.mag,'r')

            plt.sca(self.axis_fullError)
            plt.cla()
            plt.plot(self.ErrorOverFullMapHybrid ,'bo')
            plt.pause(1e-6)

        if Par.video:
            self.fig.savefig(self.videoframe % self.frameNumber)
            self.frameNumber += 1

    def saveVideo(self):
        if Par.video:
            os.system('avconv -y -f image2 -i %s -r 30 -vf "crop=\'in_w-mod(in_w,2)\':\'in_h-mod(in_h,2)\'" -pix_fmt yuv420p foo.mp4' % self.videoframe)

    def __del__(self):
        self.saveVideo()
