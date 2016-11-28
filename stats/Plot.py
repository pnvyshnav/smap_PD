#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import sys, rospy, rosbag
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
        box = self.axis_fullError.get_position()
        self.axis_fullError.set_position([box.x0, box.y0, box.width * 0.8, box.height])
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
        self.OutsideLogOddsConfidenceHybrid = []

        self.steps = []

        self.errorEvolutionsBelief = {}
        self.errorEvolutionsLogOdds = {}

    def render(self):
        plt.ion()
        while True:
            self.update()
            plt.pause(0.01)
            if self.stats.step >= self.stats.maxStep:
                break

    def update(self):
        stats = self.stats
        print("Updating for step", stats.step)
        if stats.step > self.last_step + 1:
            print("Error: Skipped %i steps" % (stats.step - self.last_step - 1))
        elif stats.step == self.last_step:
            return
        self.last_step = stats.step
        self.steps.append(stats.step)
        self.title.set_text('step = %d' % stats.step)

        # errorHybrid = np.array(stats.errorBelief)
        # errorLogOdds = np.array(stats.errorLogOdds)

        # stdVectorHybrid = np.array(stats.stdBelief)
        # stdVectorLogOdds = np.array(stats.stdLogOdds)

        # self.ErrorOverFullMapHybrid.append(np.sum(np.abs(errorHybrid))/len(errorHybrid))
        # self.ErrorOverFullMapLogOdds.append(np.sum(np.abs(errorLogOdds))/len(errorLogOdds))

        # ind_large = np.nonzero((abs(errorHybrid)>self.largeErrorThreshold) | (abs(errorLogOdds)>self.largeErrorThreshold))
        # largeErrorsHybrid = errorHybrid[ind_large]
        # largeErrorsLogodds = errorLogOdds[ind_large]
        # largeErrVarianceHybrid = stdVectorHybrid[ind_large]
        # largeErrVarianceLogodds = stdVectorLogOdds[ind_large]

        # self.OutsideConfidenceHybrid.append(len(list(filter(lambda (e, std): abs(e) > self.mag*std, zip(stats.errorBelief, stats.stdBelief)))))
        # self.OutsideConfidenceLogOdds.append(0.1+len(list(filter(lambda (e, std): abs(e) > self.mag*std, zip(stats.errorLogOdds, stats.stdLogOdds)))))
        # self.OutsideHybridConfidenceLogOdds.append(len(list(filter(lambda (e, std): abs(e) > self.mag*std, zip(stats.errorLogOdds, stats.stdBelief)))))
        # self.OutsideLogOddsConfidenceHybrid.append(0.1+len(list(filter(lambda (e, std): abs(e) > self.mag*std, zip(stats.errorBelief, stats.stdLogOdds)))))

        # plt.sca(self.axisHybrid)
        # plt.cla()
        # plt.title("SMAP")
        # plt.plot(errorHybrid)
        # plt.plot(-stdVectorHybrid*self.mag, 'r')
        # plt.plot(stdVectorHybrid*self.mag, 'r')
        #
        # plt.sca(self.axis_logOdds)
        # plt.cla()
        # plt.title("Log Odds")
        # plt.plot(errorLogOdds)
        # plt.plot(-stdVectorLogOdds*self.mag, 'r')
        # plt.plot(stdVectorLogOdds*self.mag, 'r')
        # plt.pause(1e-6)

        # plt.sca(self.axisSparseHybrid)
        # plt.cla()
        # plt.title("SMAP")
        # plt.plot(errorHybrid[::100])
        # plt.plot(-stdVectorHybrid[::100] * self.mag, 'r')
        # plt.plot(stdVectorHybrid[::100] * self.mag, 'r')
        #
        # plt.sca(self.axisSparseLogOdds)
        # plt.cla()
        # plt.title("Log Odds")
        # plt.plot(errorLogOdds[::100])
        # plt.plot(-stdVectorLogOdds[::100] * self.mag, 'r')
        # plt.plot(stdVectorLogOdds[::100] * self.mag, 'r')
        # plt.pause(1e-6)

        # plt.sca(self.axisHybridLargeErrors)
        # plt.cla()
        # plt.title("SMAP")
        # plt.plot(largeErrorsHybrid)
        # plt.plot(-largeErrVarianceHybrid*self.mag,'r')
        # plt.plot(largeErrVarianceHybrid*self.mag,'r')
        #
        # plt.sca(self.axisLogoddsLargeErrors)
        # plt.cla()
        # plt.title("Log Odds")
        # plt.plot(largeErrorsLogodds)
        # plt.plot(-largeErrVarianceLogodds*self.mag,'r')
        # plt.plot(largeErrVarianceLogodds*self.mag,'r')
        # plt.pause(1e-6)

        beliefName = 'SMAP (noise std {})'.format(stats.noiseStd)
        self.errorEvolutionsBelief[beliefName] = stats.errorEvolutionBelief
        logOddsName = 'Log Odds (noise std {s.noiseStd} inc {s.ismIncrement}, ramp {s.ismRampSize}, top {s.ismTopSize}, rslope {s.ismRampSlope})'.format(
            s=stats)
        self.errorEvolutionsLogOdds[logOddsName] = stats.errorEvolutionLogOdds

        stdBeliefStd = []
        stdLogOddsStd = []
        for step in range(stats.step):
            stdBeliefStd.append(np.mean(np.array(stats.stdCompleteBelief[step*stats.voxels:(step+1)*stats.voxels])))
            stdLogOddsStd.append(np.mean(np.array(stats.stdCompleteLogOdds[step*stats.voxels:(step+1)*stats.voxels])))
        stdBeliefStd = np.array(stdBeliefStd)
        stdLogOddsStd = np.array(stdLogOddsStd)

        plt.sca(self.axis_fullError)
        plt.cla()
        plt.title("Full Average L1-Error")
        for label, errors in sorted(self.errorEvolutionsBelief.items(), key=lambda x: x[0]):
            plt.plot(errors, label=label)
        for label, errors in sorted(self.errorEvolutionsLogOdds.items(), key=lambda x: x[0]):
            plt.plot(errors, label=label, linestyle='dashed')
        plt.plot(np.array(stats.errorEvolutionBelief) + stdBeliefStd / 100.)
        plt.plot(np.array(stats.errorEvolutionLogOdds) + stdLogOddsStd / 100.)
        plt.plot(np.array(stats.errorEvolutionBelief) - stdBeliefStd / 100.)
        plt.plot(np.array(stats.errorEvolutionLogOdds) - stdLogOddsStd / 100.)
        plt.legend(loc='upper left', bbox_to_anchor=(1, .5),
                   ncol=1, fancybox=True, shadow=True).draggable()
        plt.pause(1e-6)

        # plt.sca(self.axis_outside)
        # plt.cla()
        # plt.title("Errors Outside 2 Stddev Interval")
        # self.axis_outside.set_yscale('log')
        # plt.scatter(self.steps, self.OutsideConfidenceHybrid, c='#ff0000', edgecolor='#ff0000', marker='*', label="SMAP")
        # plt.scatter(self.steps, self.OutsideLogOddsConfidenceHybrid, c='#999000', edgecolor='#999000', marker='*', linewidths=5, label="SMAP (Log Odds stddev)")
        # plt.scatter(self.steps, self.OutsideConfidenceLogOdds, c='#0000ff', edgecolor='#0000ff', marker='o', label="Log Odds")
        # plt.scatter(self.steps, self.OutsideHybridConfidenceLogOdds, c='#009099', edgecolor='#009099', marker='o', label="Log Odds (SMAP stddev)")
        # plt.legend(loc=2)
        # plt.pause(1e-6)

        plt.draw()


results = Results()


def callback(data):
    global results
    results.stats = data


def main(argv):
    global results

    if len(argv) <= 1:
        rospy.init_node('smap_stats')
        rospy.Subscriber("stats", smapStats, callback)
    else:
        # load bag file from argv[1]
        bag = rosbag.Bag(argv[1])
        for topic, msg, t in bag.read_messages(topics=['stats']):
            results.stats = msg
        bag.close()
        print("Loaded ROS bag file from %s." % argv[1])

    results.render()

    plt.ioff()
    plt.show()


if __name__ == '__main__':
    main(sys.argv)
