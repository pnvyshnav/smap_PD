#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import sys, rospy
from smap.msg import smapStats


class Results:
    def __init__(self):
        self.largeErrorThreshold = 0.6
        self.frameNumber = 0
        fh = plt.figure('Inconsistencies')
        self.fig = fh
        self.axisHybrid = fh.add_subplot(211)
        self.axis_logOdds = fh.add_subplot(212)
        self.fig_fullError = plt.figure('FullError')
        self.axis_fullError = self.fig_fullError.add_subplot(111)
        self.fig_zoomedInconsistency = plt.figure('zoomedOnInconsistencies')
        self.axisHybridLargeErrors = self.fig_zoomedInconsistency.add_subplot(211)
        self.axisLogoddsLargeErrors = self.fig_zoomedInconsistency.add_subplot(212)

        self.fig_sparse = plt.figure('Sparse Inconsistencies')
        self.axisSparseHybrid = self.fig_sparse.add_subplot(211)
        self.axisSparseLogOdds = self.fig_sparse.add_subplot(212)

        self.fig_outside = plt.figure('Deviations > 2 Stddev')
        self.axis_outside = self.fig_outside.add_subplot(111)

        self.mag = 2
        self.errorHybridCurve = None

        self.stats = smapStats()
        self.title = self.fig.suptitle('step = %d' % self.stats.step)

        self.last_step = 0

        self.ErrorOverFullMapHybrid = []
        self.ErrorOverFullMapLogOdds = []

        self.OutsideConfidenceHybrid = []
        self.OutsideConfidenceLogOdds = []
        self.OutsideHybridConfidenceLogOdds = []


    def render(self):
        plt.ion()
        while True:
            self.update()
            plt.pause(0.01)

    def update(self):
        stats = self.stats
        print "Updating for step", stats.step
        if stats.step > self.last_step + 1:
            print "Error: Skipped %i steps" % (stats.step - self.last_step - 1)
        elif stats.step == self.last_step:
            return
        self.last_step = stats.step
        self.title.set_text('step = %d' % stats.step)


        # window = plt.get_current_fig_manager().window
        # w = window.winfo_screenwidth()
        # h = window.winfo_screenheight()
        # window.wm_geomtery('=%dx%d' % (w, h))
        # plt.subplots_adjust(left=0.05, right=0.95, bottom=0.05, top=0.95, wspace=0, hspace=0)

        errorHybrid = np.array(stats.errorBelief)
        errorLogOdds = np.array(stats.errorLogOdds)

        stdVectorHybrid = np.array(stats.stdBelief)
        stdVectorLogOdds = np.array(stats.stdLogOdds)

        self.ErrorOverFullMapHybrid.append(np.sum(np.abs(errorHybrid))/len(errorHybrid))
        self.ErrorOverFullMapLogOdds.append(np.sum(np.abs(errorLogOdds))/len(errorLogOdds))

        ind_large = np.nonzero((abs(errorHybrid)>self.largeErrorThreshold) | (abs(errorLogOdds)>self.largeErrorThreshold))
        largeErrorsHybrid = errorHybrid[ind_large]
        largeErrorsLogodds = errorLogOdds[ind_large]
        largeErrVarianceHybrid = stdVectorHybrid[ind_large]
        largeErrVarianceLogodds = stdVectorLogOdds[ind_large]

        self.OutsideConfidenceHybrid.append(len(list(filter(lambda (e, std): abs(e) > self.mag*std, zip(stats.errorBelief, stats.stdBelief)))))
        self.OutsideConfidenceLogOdds.append(len(list(filter(lambda (e, std): abs(e) > self.mag*std, zip(stats.errorLogOdds, stats.stdLogOdds)))))
        self.OutsideHybridConfidenceLogOdds.append(len(list(filter(lambda (e, std): abs(e) > self.mag*std, zip(stats.errorLogOdds, stats.stdBelief)))))

        plt.sca(self.axisHybrid)
        plt.cla()
        plt.title("SMAP")
        plt.plot(errorHybrid)
        plt.plot(-stdVectorHybrid*self.mag, 'r')
        plt.plot(stdVectorHybrid*self.mag, 'r')

        plt.sca(self.axis_logOdds)
        plt.cla()
        plt.title("Log Odds")
        plt.plot(errorLogOdds)
        plt.plot(-stdVectorLogOdds*self.mag, 'r')
        plt.plot(stdVectorLogOdds*self.mag, 'r')
        plt.pause(1e-6)

        plt.sca(self.axisSparseHybrid)
        plt.cla()
        plt.title("SMAP")
        plt.plot(errorHybrid[::100])
        plt.plot(-stdVectorHybrid[::100] * self.mag, 'r')
        plt.plot(stdVectorHybrid[::100] * self.mag, 'r')

        plt.sca(self.axisSparseLogOdds)
        plt.cla()
        plt.title("Log Odds")
        plt.plot(errorLogOdds[::100])
        plt.plot(-stdVectorLogOdds[::100] * self.mag, 'r')
        plt.plot(stdVectorLogOdds[::100] * self.mag, 'r')
        plt.pause(1e-6)

        plt.sca(self.axisHybridLargeErrors)
        plt.cla()
        plt.title("SMAP")
        plt.plot(largeErrorsHybrid)
        plt.plot(-largeErrVarianceHybrid*self.mag,'r')
        plt.plot(largeErrVarianceHybrid*self.mag,'r')

        plt.sca(self.axisLogoddsLargeErrors)
        plt.cla()
        plt.title("Log Odds")
        plt.plot(largeErrorsLogodds)
        plt.plot(-largeErrVarianceLogodds*self.mag,'r')
        plt.plot(largeErrVarianceLogodds*self.mag,'r')
        plt.pause(1e-6)

        plt.sca(self.axis_fullError)
        plt.cla()
        plt.title("Full Average L1-Error")
        plt.plot(self.ErrorOverFullMapHybrid, c='b', marker='o', label="SMAP")
        plt.plot(self.ErrorOverFullMapLogOdds, c='r', marker='*', label="Log Odds")
        plt.legend()
        plt.pause(1e-6)

        plt.sca(self.axis_outside)
        plt.cla()
        plt.title("Errors Outside 2 Stddev Interval")
        plt.scatter(list(range(1, len(self.OutsideConfidenceHybrid)+1)), self.OutsideConfidenceHybrid, c='b', marker='*', label="SMAP")
        plt.scatter(list(range(1, len(self.OutsideConfidenceHybrid)+1)), self.OutsideConfidenceLogOdds, c='r', marker='o', label="Log Odds")
        plt.scatter(list(range(1, len(self.OutsideConfidenceHybrid)+1)), self.OutsideHybridConfidenceLogOdds, c='g', marker='o', label="Log Odds (SMAP stddev)")
        plt.legend(loc=2)
        plt.pause(1e-6)

        plt.draw()


results = Results()

def callback(data):
    global results
    results.stats = data

def main(argv):
    global results

    rospy.init_node('smap_stats')
    rospy.Subscriber("stats", smapStats, callback)

    results.render()

if __name__ == '__main__':
    main(sys.argv)
