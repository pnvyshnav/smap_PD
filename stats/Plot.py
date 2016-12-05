#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import sys, rospy, rosbag
from smap.msg import smapStats

import matplotlib.cm as cm
from matplotlib.colors import LogNorm


class Results:
    def __init__(self):
        self.largeErrorThreshold = 0.6
        self.frameNumber = 0
        fh = plt.figure('Inconsistencies at first step')
        self.fig = fh
        self.axisHybrid = fh.add_subplot(211)
        self.axis_logOdds = fh.add_subplot(212)

        self.lastStepFig = plt.figure('Inconsistencies at last step')
        self.axisLastHybrid = self.lastStepFig.add_subplot(211)
        self.axisLast_logOdds = self.lastStepFig.add_subplot(212)
        self.fig_fullError = plt.figure('Mean Absolute Error (MAE) Evolution over all Voxels')
        self.axis_fullError = self.fig_fullError.add_subplot(111)
        box = self.axis_fullError.get_position()
        self.axis_fullError.set_position([box.x0, box.y0, box.width * 0.8, box.height])
        self.fig_zoomedInconsistency = plt.figure('zoomedOnInconsistencies')
        self.axisHybridLargeErrors = self.fig_zoomedInconsistency.add_subplot(211)
        self.axisLogoddsLargeErrors = self.fig_zoomedInconsistency.add_subplot(212)

        self.fig_errorStdDiff = plt.figure('Difference from Error to Std')
        self.axisErrorStdDiff1 = self.fig_errorStdDiff.add_subplot(311)
        self.axisErrorStdDiff2 = self.fig_errorStdDiff.add_subplot(312)
        self.axisErrorStdDiff3 = self.fig_errorStdDiff.add_subplot(313)

        self.fig_inconsistencies = plt.figure('Errors Outside Std Interval')
        self.axisInconsistencies1 = self.fig_inconsistencies.add_subplot(311)
        self.axisInconsistencies2 = self.fig_inconsistencies.add_subplot(312)
        self.axisInconsistencies3 = self.fig_inconsistencies.add_subplot(313)

        self.fig_roundedInconsistencies = plt.figure('Rounded Errors Outside Std Interval')
        self.axisRoundedInconsistencies = self.fig_roundedInconsistencies.add_subplot(111)

        self.fig_sparse = plt.figure('Sparse Inconsistencies')
        self.axisSparseHybrid = self.fig_sparse.add_subplot(211)
        self.axisSparseLogOdds = self.fig_sparse.add_subplot(212)

        self.fig_outside = plt.figure('Deviations > 2 Stddev')
        self.axis_outside = self.fig_outside.add_subplot(111)

        self.fig_histogram = plt.figure('Error Histogram Evolution')
        self.axisHistogramHybrid = self.fig_histogram.add_subplot(121)
        self.axisHistogramLogOdds = self.fig_histogram.add_subplot(122)

        self.fig_rmse = plt.figure('Root Mean Square Error (RMSE) Evolution over all Voxels')
        self.axis_rmse = self.fig_rmse.add_subplot(111)

        self.mag = 2
        self.errorHybridCurve = None

        self.stats = smapStats()
        self.fig.suptitle('Mean and std dev of updated voxels at first step')
        self.lastStepFig.suptitle('Mean and std dev of updated voxels at last step')

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

        beliefName = 'SMAP (noise std {})'.format(stats.noiseStd)
        self.errorEvolutionsBelief[beliefName] = stats.errorEvolutionBelief
        logOddsName = 'Log Odds (noise std {s.noiseStd} inc {s.ismIncrement}, ramp {s.ismRampSize}, top {s.ismTopSize}, rslope {s.ismRampSlope})'.format(
            s=stats)
        self.errorEvolutionsLogOdds[logOddsName] = stats.errorEvolutionLogOdds

        plt.sca(self.axisRoundedInconsistencies)
        plt.cla()
        plt.title("Inconsistencies Rounded Error to 1 Std Deviation")
        inconsistentBelief = []
        inconsistentLogOdds = []
        start = 0
        for step in range(stats.step):
            inconsistentBelief.append(len(list(filter(
                lambda (e, std): round(abs(e)) > std,
                zip(stats.errorCompleteUpdatedBelief[start:start + stats.updatedVoxels[step]],
                    stats.stdCompleteUpdatedBelief[start:start + stats.updatedVoxels[step]])
            ))) * 1. / stats.updatedVoxels[step])
            inconsistentLogOdds.append(len(list(filter(
                lambda (e, std): round(abs(e)) > std,
                zip(stats.errorCompleteUpdatedLogOdds[start:start + stats.updatedVoxels[step]],
                    stats.stdCompleteUpdatedLogOdds[start:start + stats.updatedVoxels[step]])
            ))) * 1. / stats.updatedVoxels[step])
            start += stats.updatedVoxels[step]
        plt.plot(inconsistentBelief, label=beliefName)
        plt.plot(inconsistentLogOdds, label=logOddsName)
        plt.legend(loc='upper right', bbox_to_anchor=(1, .5),
                   ncol=1, fancybox=True, shadow=True).draggable()

        plt.sca(self.axisInconsistencies1)
        plt.cla()
        plt.title("Inconsistencies Error to 1 Std Deviation")
        inconsistentBelief = []
        inconsistentLogOdds = []
        start = 0
        for step in range(stats.step):
            inconsistentBelief.append(len(list(filter(
                lambda (e, std): abs(e) > std,
                zip(stats.errorCompleteUpdatedBelief[start:start + stats.updatedVoxels[step]],
                    stats.stdCompleteUpdatedBelief[start:start + stats.updatedVoxels[step]])
            ))) * 1. / stats.updatedVoxels[step])
            inconsistentLogOdds.append(len(list(filter(
                lambda (e, std): abs(e) > std,
                zip(stats.errorCompleteUpdatedLogOdds[start:start + stats.updatedVoxels[step]],
                    stats.stdCompleteUpdatedLogOdds[start:start + stats.updatedVoxels[step]])
            ))) * 1. / stats.updatedVoxels[step])
            start += stats.updatedVoxels[step]
        plt.plot(inconsistentBelief, label=beliefName)
        plt.plot(inconsistentLogOdds, label=logOddsName)
        plt.legend(loc='upper right', bbox_to_anchor=(1, .5),
                   ncol=1, fancybox=True, shadow=True).draggable()

        plt.sca(self.axisInconsistencies2)
        plt.cla()
        plt.title("Inconsistencies Error to 2 Std Deviations")
        inconsistentBelief = []
        inconsistentLogOdds = []
        start = 0
        for step in range(stats.step):
            inconsistentBelief.append(len(list(filter(
                lambda (e, std): abs(e) > 2. * std,
                zip(stats.errorCompleteUpdatedBelief[start:start + stats.updatedVoxels[step]],
                    stats.stdCompleteUpdatedBelief[start:start + stats.updatedVoxels[step]])
            ))) * 1. / stats.updatedVoxels[step])
            inconsistentLogOdds.append(len(list(filter(
                lambda (e, std): abs(e) > 2. * std,
                zip(stats.errorCompleteUpdatedLogOdds[start:start + stats.updatedVoxels[step]],
                    stats.stdCompleteUpdatedLogOdds[start:start + stats.updatedVoxels[step]])
            ))) * 1. / stats.updatedVoxels[step])
            start += stats.updatedVoxels[step]
        plt.plot(inconsistentBelief, label=beliefName)
        plt.plot(inconsistentLogOdds, label=logOddsName)
        plt.legend(loc='upper right', bbox_to_anchor=(1, .5),
                   ncol=1, fancybox=True, shadow=True).draggable()

        plt.sca(self.axisInconsistencies3)
        plt.cla()
        plt.title("Inconsistencies Error to 3 Std Deviations")
        inconsistentBelief = []
        inconsistentLogOdds = []
        start = 0
        for step in range(stats.step):
            inconsistentBelief.append(len(list(filter(
                lambda (e, std): abs(e) > 3. * std,
                zip(stats.errorCompleteUpdatedBelief[start:start + stats.updatedVoxels[step]],
                    stats.stdCompleteUpdatedBelief[start:start + stats.updatedVoxels[step]])
            ))) * 1. / stats.updatedVoxels[step])
            inconsistentLogOdds.append(len(list(filter(
                lambda (e, std): abs(e) > 3. * std,
                zip(stats.errorCompleteUpdatedLogOdds[start:start + stats.updatedVoxels[step]],
                    stats.stdCompleteUpdatedLogOdds[start:start + stats.updatedVoxels[step]])
            ))) * 1. / stats.updatedVoxels[step])
            start += stats.updatedVoxels[step]
        plt.plot(inconsistentBelief, label=beliefName)
        plt.plot(inconsistentLogOdds, label=logOddsName)
        plt.legend(loc='upper right', bbox_to_anchor=(1, .5),
                   ncol=1, fancybox=True, shadow=True).draggable()

        plt.sca(self.axisErrorStdDiff1)
        plt.cla()
        plt.title("Mean Difference from Error to 1 Std Deviation")
        errStdDiffBelief = []
        errStdDiffLogOdds = []
        start = 0
        for step in range(stats.step):
            errStdDiffBelief.append(np.mean(
                np.array(stats.stdCompleteUpdatedBelief[start:start + stats.updatedVoxels[step]]) -
                np.array(stats.errorCompleteUpdatedBelief[start:start + stats.updatedVoxels[step]])
            ))
            errStdDiffLogOdds.append(np.mean(
                np.array(stats.stdCompleteUpdatedLogOdds[start:start + stats.updatedVoxels[step]]) -
                np.array(stats.errorCompleteUpdatedLogOdds[start:start + stats.updatedVoxels[step]])
            ))
            start += stats.updatedVoxels[step]

        plt.plot(errStdDiffBelief, label=beliefName)
        plt.plot(errStdDiffLogOdds, label=logOddsName)
        plt.legend(loc='upper right', bbox_to_anchor=(1, .5),
                   ncol=1, fancybox=True, shadow=True).draggable()

        plt.sca(self.axisErrorStdDiff2)
        plt.cla()
        plt.title("Mean Difference from Error to 2 Std Deviations")
        errStdDiffBelief = []
        errStdDiffLogOdds = []
        start = 0
        for step in range(stats.step):
            errStdDiffBelief.append(np.mean(
                np.array(stats.stdCompleteUpdatedBelief[start:start + stats.updatedVoxels[step]]) * 2. -
                np.array(stats.errorCompleteUpdatedBelief[start:start + stats.updatedVoxels[step]])
            ))
            errStdDiffLogOdds.append(np.mean(
                np.array(stats.stdCompleteUpdatedLogOdds[start:start + stats.updatedVoxels[step]]) * 2. -
                np.array(stats.errorCompleteUpdatedLogOdds[start:start + stats.updatedVoxels[step]])
            ))
            start += stats.updatedVoxels[step]

        plt.plot(errStdDiffBelief, label=beliefName)
        plt.plot(errStdDiffLogOdds, label=logOddsName)
        plt.legend(loc='upper right', bbox_to_anchor=(1, .5),
                   ncol=1, fancybox=True, shadow=True).draggable()

        plt.sca(self.axisErrorStdDiff3)
        plt.cla()
        plt.title("Mean Difference from Error to 3 Std Deviations")
        errStdDiffBelief = []
        errStdDiffLogOdds = []
        start = 0
        for step in range(stats.step):
            errStdDiffBelief.append(np.mean(
                np.array(stats.stdCompleteUpdatedBelief[start:start + stats.updatedVoxels[step]]) * 3. -
                np.array(stats.errorCompleteUpdatedBelief[start:start + stats.updatedVoxels[step]])
            ))
            errStdDiffLogOdds.append(np.mean(
                np.array(stats.stdCompleteUpdatedLogOdds[start:start + stats.updatedVoxels[step]]) * 3. -
                np.array(stats.errorCompleteUpdatedLogOdds[start:start + stats.updatedVoxels[step]])
            ))
            start += stats.updatedVoxels[step]

        plt.plot(errStdDiffBelief, label=beliefName)
        plt.plot(errStdDiffLogOdds, label=logOddsName)
        plt.legend(loc='upper right', bbox_to_anchor=(1, .5),
                   ncol=1, fancybox=True, shadow=True).draggable()

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

        ratioBelief = len(list(filter(
            lambda (e, std): abs(e) > self.mag * std,
            zip(stats.errorCompleteUpdatedBelief[-stats.updatedVoxels[-1]:],
                stats.stdCompleteUpdatedBelief[-stats.updatedVoxels[-1]:])
        ))) * 1. / stats.updatedVoxels[-1]
        print(list(filter(
            lambda (e, std): abs(e) > self.mag * std,
            zip(stats.errorCompleteUpdatedBelief[-stats.updatedVoxels[-1]:],
                stats.stdCompleteUpdatedBelief[-stats.updatedVoxels[-1]:])
        )))
        ratioLogOdds = len(list(filter(
            lambda (e, std): abs(e) > self.mag * std,
            zip(stats.errorCompleteUpdatedLogOdds[-stats.updatedVoxels[-1]:],
                stats.stdCompleteUpdatedLogOdds[-stats.updatedVoxels[-1]:])
        ))) * 1. / stats.updatedVoxels[-1]

        stdLastBelief = np.array(stats.stdCompleteUpdatedBelief[-stats.updatedVoxels[-1]:])
        stdLastLogOdds = np.array(stats.stdCompleteUpdatedLogOdds[-stats.updatedVoxels[-1]:])
        plt.sca(self.axisLastHybrid)
        plt.cla()
        plt.title("SMAP")
        self.axisLastHybrid.set_xlim([0, stats.updatedVoxels[-1]])


        plt.plot(stats.errorCompleteUpdatedBelief[-stats.updatedVoxels[-1]:])
        plt.plot(-stdLastBelief * self.mag, 'r')
        plt.plot(stdLastBelief * self.mag, 'r')

        # add scatter of inconsistencies
        lb = np.array(-stdLastBelief * self.mag)
        ub = np.array(stdLastBelief * self.mag)
        err = np.array(stats.errorCompleteUpdatedBelief[-stats.updatedVoxels[-1]:])
        inconsistencies = err[(err > ub) | (err < lb)]
        indices = np.array(list(range(stats.updatedVoxels[-1])))
        indices = indices[(err > ub) | (err < lb)]
        plt.scatter(indices, inconsistencies, c='g', edgecolor='g')

        plt.sca(self.axisLast_logOdds)
        plt.cla()
        plt.title("Log Odds")
        self.axisLast_logOdds.set_xlim([0, stats.updatedVoxels[-1]])
        plt.plot(stats.errorCompleteUpdatedLogOdds[-stats.updatedVoxels[-1]:])
        plt.plot(-stdLastLogOdds * self.mag, 'r')
        plt.plot(stdLastLogOdds * self.mag, 'r')

        # add scatter of inconsistencies
        lb = np.array(-stdLastLogOdds * self.mag)
        ub = np.array(stdLastLogOdds * self.mag)
        err = np.array(stats.errorCompleteUpdatedLogOdds[-stats.updatedVoxels[-1]:])
        inconsistencies = err[(err > ub) | (err < lb)]
        indices = np.array(list(range(stats.updatedVoxels[-1])))
        indices = indices[(err > ub) | (err < lb)]
        plt.scatter(indices, inconsistencies, c='g', edgecolor='g')

        plt.pause(1e-6)

        ratioBelief = len(list(filter(
            lambda (e, std): abs(e) > self.mag * std,
            zip(stats.errorCompleteUpdatedBelief[:stats.updatedVoxels[0]],
                stats.stdCompleteUpdatedBelief[:stats.updatedVoxels[0]])
        ))) * 1. / stats.updatedVoxels[-1]
        print(list(filter(
            lambda (e, std): abs(e) > self.mag * std,
            zip(stats.errorCompleteUpdatedBelief[:stats.updatedVoxels[0]],
                stats.stdCompleteUpdatedBelief[:stats.updatedVoxels[0]])
        )))
        ratioLogOdds = len(list(filter(
            lambda (e, std): abs(e) > self.mag * std,
            zip(stats.errorCompleteUpdatedLogOdds[:stats.updatedVoxels[0]],
                stats.stdCompleteUpdatedLogOdds[:stats.updatedVoxels[0]])
        ))) * 1. / stats.updatedVoxels[-1]

        stdLastBelief = np.array(stats.stdCompleteUpdatedBelief[:stats.updatedVoxels[0]])
        stdLastLogOdds = np.array(stats.stdCompleteUpdatedLogOdds[:stats.updatedVoxels[0]])

        plt.sca(self.axisHybrid)
        plt.cla()
        plt.title("SMAP")
        plt.plot(stats.errorCompleteUpdatedBelief[:stats.updatedVoxels[0]])
        self.axisHybrid.set_xlim([0, stats.updatedVoxels[0]])
        plt.plot(-stdLastBelief * self.mag, 'r')
        plt.plot(stdLastBelief * self.mag, 'r')

        # add scatter of inconsistencies
        lb = np.array(-stdLastBelief * self.mag)
        ub = np.array(stdLastBelief * self.mag)
        err = np.array(stats.errorCompleteUpdatedBelief[:stats.updatedVoxels[0]])
        inconsistencies = err[(err > ub) | (err < lb)]
        indices = np.array(list(range(stats.updatedVoxels[0])))
        indices = indices[(err > ub) | (err < lb)]
        plt.scatter(indices, inconsistencies, c='g', edgecolor='g')


        plt.sca(self.axis_logOdds)
        plt.cla()
        plt.title("Log Odds")
        plt.plot(stats.errorCompleteUpdatedLogOdds[:stats.updatedVoxels[0]])
        self.axis_logOdds.set_xlim([0, stats.updatedVoxels[0]])
        plt.plot(-stdLastLogOdds * self.mag, 'r')
        plt.plot(stdLastLogOdds * self.mag, 'r')

        # add scatter of inconsistencies
        lb = np.array(-stdLastLogOdds * self.mag)
        ub = np.array(stdLastLogOdds * self.mag)
        err = np.array(stats.errorCompleteUpdatedLogOdds[:stats.updatedVoxels[0]])
        inconsistencies = err[(err > ub) | (err < lb)]
        indices = np.array(list(range(stats.updatedVoxels[0])))
        indices = indices[(err > ub) | (err < lb)]
        plt.scatter(indices, inconsistencies, c='g', edgecolor='g')

        plt.pause(1e-6)

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

        plt.sca(self.axis_fullError)
        plt.cla()
        plt.title("Mean Absolute Error (MAE) Evolution over all Voxels")

        for label, errors in sorted(self.errorEvolutionsBelief.items(), key=lambda x: x[0]):
            plt.plot(errors, label=label)
        for label, errors in sorted(self.errorEvolutionsLogOdds.items(), key=lambda x: x[0]):
            plt.plot(errors, label=label, linestyle='dashed')
        # plt.plot(np.array(stats.errorEvolutionBelief) + stdBeliefStd / 100.)
        # plt.plot(np.array(stats.errorEvolutionLogOdds) + stdLogOddsStd / 100.)
        # plt.plot(np.array(stats.errorEvolutionBelief) - stdBeliefStd / 100.)
        # plt.plot(np.array(stats.errorEvolutionLogOdds) - stdLogOddsStd / 100.)

        print('Original Size:', len(stats.errorCompleteBelief))
        # errorCompleteBelief = np.abs(np.array(stats.errorCompleteBelief[:int(steps*stats.voxels)])).reshape((-1, steps))
        # print('Size:', errorCompleteBelief.shape)
        # for step in range(steps):
        #     plt.scatter([step]*stats.voxels, errorCompleteBelief[:,step])
        plt.legend(loc='upper left', bbox_to_anchor=(1, .5),
                   ncol=1, fancybox=True, shadow=True).draggable()
        plt.pause(1e-6)

        plt.sca(self.axis_rmse)
        plt.cla()
        # plt.title("Root Mean Square Error (RMSE) Evolution")
        # rmseBelief = np.mean(np.sqrt(
        #     np.square(np.array(stats.errorCompleteBelief)) + np.square(np.array(stats.stdCompleteBelief))).reshape(
        #     (-1, stats.voxels)), 1)
        # rmseLogOdds = np.mean(np.sqrt(
        #     np.square(np.array(stats.errorCompleteLogOdds)) + np.square(np.array(stats.stdCompleteLogOdds))).reshape(
        #     (-1, stats.voxels)), 1)
        # plt.plot(rmseBelief, label=beliefName)
        # plt.plot(rmseLogOdds, label=logOddsName, linestyle='dashed')
        # plt.legend(loc='upper right', bbox_to_anchor=(1, .5),
        #            ncol=1, fancybox=True, shadow=True).draggable()

        plt.title("Root Mean Square Error (RMSE) Evolution of Updated Voxels until Obstacle")
        rmseBelief = []
        rmseLogOdds = []
        start = 0
        for step in range(stats.step):
            rmseBelief.append(np.mean(np.sqrt(
                np.square(np.array(stats.errorCompleteUpdatedBelief[start:start + stats.updatedVoxels[step]])) +
                np.square(np.array(stats.stdCompleteUpdatedBelief[start:start + stats.updatedVoxels[step]]))
            )))
            rmseLogOdds.append(np.mean(np.sqrt(
                np.square(np.array(stats.errorCompleteUpdatedLogOdds[start:start + stats.updatedVoxels[step]])) +
                np.square(np.array(stats.stdCompleteUpdatedLogOdds[start:start + stats.updatedVoxels[step]]))
            )))
            start += stats.updatedVoxels[step]

        plt.plot(rmseBelief, label=beliefName)
        plt.plot(rmseLogOdds, label=logOddsName)
        plt.legend(loc='upper right', bbox_to_anchor=(1, .5),
                   ncol=1, fancybox=True, shadow=True).draggable()

        bins = 1000
        miny = 0.0
        maxy = 1.0
        steps = stats.step

        plt.sca(self.axisHistogramHybrid)
        plt.cla()
        plt.title("SMAP Error Histogram Evolution of Updated Voxels until Obstacle")

        total = []
        start = 0
        averageErrorUpdatedBelief = []
        for step in range(steps):
            b = [0.0001] * bins
            sum = 0
            for error in stats.errorCompleteUpdatedBelief[start:start + stats.updatedVoxels[step]]:
                error = abs(error)
                # if abs(error - 0.5) < 1e-2:
                #     continue
                # print error
                bin_number = int(round(bins * (1 - (error - miny) / (maxy - miny))))
                if 0 <= bin_number < bins:
                    b[bin_number] += 1  # TODO bin number incorrect?
                    # if bin_number != 50:
                    #     print bin_number
                sum += error
            start += stats.updatedVoxels[step]
            total += b
            averageErrorUpdatedBelief.append(sum / stats.updatedVoxels[step])
            # stdLogOddsStd.append(np.mean(np.array(stats.stdCompleteLogOdds[step*stats.voxels:(step+1)*stats.voxels])))
        # stdBeliefStd = np.array(stdBeliefStd)
        # stdLogOddsStd = np.array(stdLogOddsStd)

        img = np.array(total)
        img = img.reshape((steps, -1))
        img = img.transpose()
        print img.shape
        plt.imshow(img, aspect='auto', interpolation='none', extent=[0, steps, miny, maxy], norm=LogNorm())
        plt.colorbar()
        # plt.plot(stats.errorEvolutionBelief, 'w')
        plt.plot(averageErrorUpdatedBelief, 'w')
        plt.pause(1e-6)

        plt.sca(self.axisHistogramLogOdds)
        plt.cla()
        plt.title("Log-Odds Error Histogram Evolution of Updated Voxels until Obstacle")

        total = []
        start = 0
        averageErrorUpdatedLogOdds = []
        for step in range(steps):
            b = [0.0001] * bins
            sum = 0
            for error in stats.errorCompleteUpdatedLogOdds[start:start + stats.updatedVoxels[step]]:
                error = abs(error)
                # if abs(error - 0.5) < 1e-2:
                #     continue
                # print error
                bin_number = int(round(bins * (1 - (error - miny) / (maxy - miny))))
                if 0 <= bin_number < bins:
                    b[bin_number] += 1
                sum += error
            start += stats.updatedVoxels[step]
            total += b
            averageErrorUpdatedLogOdds.append(sum / stats.updatedVoxels[step])

        img = np.array(total)
        img = img.reshape((steps, -1))
        img = img.transpose()
        print img.shape
        plt.imshow(img, aspect='auto', interpolation='none', extent=[0, steps, miny, maxy], norm=LogNorm())
        plt.colorbar()
        # plt.plot(stats.errorEvolutionLogOdds, 'w')
        plt.plot(averageErrorUpdatedLogOdds, 'w')

        # total = []
        # stdBeliefStd = []
        # stdLogOddsStd = []
        # for step in range(steps):
        #     b = [0.0001] * bins
        #     for error in stats.errorCompleteLogOdds[step * stats.voxels:(step + 1) * stats.voxels]:
        #         error = abs(error)
        #         # if abs(error - 0.5) < 1e-2:
        #         #     continue
        #         # print error
        #         bin_number = int(round(bins * (1 - (error - miny) / (maxy - miny))))
        #         if 0 <= bin_number < bins:
        #             b[bin_number] += 1  # TODO bin number incorrect?
        #             # if bin_number != 50:
        #             #     print bin_number
        #     total += b
        #     # stdLogOddsStd.append(np.mean(np.array(stats.stdCompleteLogOdds[step*stats.voxels:(step+1)*stats.voxels])))
        # # stdBeliefStd = np.array(stdBeliefStd)
        # # stdLogOddsStd = np.array(stdLogOddsStd)
        #
        # img = np.array(total)
        # img = img.reshape((steps, -1))
        # img = img.transpose()
        # print img.shape
        # plt.imshow(img, aspect='auto', interpolation='none', extent=[0, steps, miny, maxy], norm=LogNorm())
        # plt.plot(stats.errorEvolutionLogOdds, 'w')










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
