#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import sys, rospy, rosbag

import scipy
import scipy.stats
from scipy.stats import pearsonr

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
        self.lastStepCorrectFig = plt.figure('Correct Estimate Inconsistencies at last step')
        self.axisLastCorrectHybrid = self.lastStepCorrectFig.add_subplot(211)
        self.axisLastCorrect_logOdds = self.lastStepCorrectFig.add_subplot(212)
        self.lastStepCorrectedFig = plt.figure('Corrected Inconsistencies at last step')
        self.axisLastCorrectedHybrid = self.lastStepCorrectedFig.add_subplot(211)
        self.axisLastCorrected_logOdds = self.lastStepCorrectedFig.add_subplot(212)
        self.firstStepCorrectedFig = plt.figure('Corrected Inconsistencies at first step')
        self.axisFirstCorrectedHybrid = self.firstStepCorrectedFig.add_subplot(211)
        self.axisFirstCorrected_logOdds = self.firstStepCorrectedFig.add_subplot(212)
        self.fig_fullError = plt.figure('Mean Absolute Error (MAE) Evolution over all Voxels')
        self.axis_fullError = self.fig_fullError.add_subplot(111)
        box = self.axis_fullError.get_position()
        self.axis_fullError.set_position([box.x0, box.y0, box.width * 0.8, box.height])

        self.fig_errorStdDiff = plt.figure('Difference from Error to Std')
        self.axisErrorStdDiff1 = self.fig_errorStdDiff.add_subplot(311)
        self.axisErrorStdDiff2 = self.fig_errorStdDiff.add_subplot(312)
        self.axisErrorStdDiff3 = self.fig_errorStdDiff.add_subplot(313)

        self.fig_inconsistencies = plt.figure('Correlation of Error and Std')
        self.axisCorrelation = self.fig_inconsistencies.add_subplot(111)

        self.fig_posNegDistance = plt.figure('Positive/Negative Distance between Error and Std')
        self.axisPosNegDistanceHybrid = self.fig_posNegDistance.add_subplot(211)
        self.axisPosNegDistanceLogOdds = self.fig_posNegDistance.add_subplot(212)

        self.fig_correctStdDev = plt.figure('Std Dev for Correct Prediction')
        self.fig_correctStdDev.suptitle("Std Dev where Prediction Error < 0.3")
        self.axisCorrectStdDevHybrid = self.fig_correctStdDev.add_subplot(211)
        self.axisCorrectStdDevLogOdds = self.fig_correctStdDev.add_subplot(212)

        self.fig_roundedInconsistencies = plt.figure('Rounded Errors Outside Std Interval')
        self.axisRoundedInconsistencies = self.fig_roundedInconsistencies.add_subplot(111)

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

        def correct(std, err):
            minStd = np.min(std)
            maxStd = np.max(std)
            dStd = maxStd - minStd
            print "std params: ", minStd, maxStd
            minErr = np.min(err)
            maxErr = np.max(err)
            dErr = maxErr - minErr
            print "err params: ", minErr, maxErr

            newStd = (std  - minStd) * dErr / dStd + minErr
            minNew = np.min(newStd)
            maxNew = np.max(newStd)
            print "new params: ", minNew, maxNew

            return newStd

        beliefName = 'SMAP (noise std {})'.format(stats.noiseStd)
        self.errorEvolutionsBelief[beliefName] = stats.errorEvolutionBelief
        logOddsName = 'Log Odds (noise std {s.noiseStd} inc {s.ismIncrement}, ramp {s.ismRampSize}, top {s.ismTopSize}, rslope {s.ismRampSlope})'.format(
            s=stats)
        self.errorEvolutionsLogOdds[logOddsName] = stats.errorEvolutionLogOdds

        # plt.sca(self.axisRoundedInconsistencies)
        # plt.cla()
        # plt.title("Inconsistencies Rounded Error to 1 Std Deviation")
        # inconsistentBelief = []
        # inconsistentLogOdds = []
        # start = 0
        # for step in range(stats.step):
        #     inconsistentBelief.append(len(list(filter(
        #         lambda (e, std): round(abs(e)) > std,
        #         zip(stats.errorCompleteUpdatedBelief[start:start + stats.updatedVoxels[step]],
        #             stats.stdCompleteUpdatedBelief[start:start + stats.updatedVoxels[step]])
        #     ))) * 1. / stats.updatedVoxels[step])
        #     inconsistentLogOdds.append(len(list(filter(
        #         lambda (e, std): round(abs(e)) > std,
        #         zip(stats.errorCompleteUpdatedLogOdds[start:start + stats.updatedVoxels[step]],
        #             stats.stdCompleteUpdatedLogOdds[start:start + stats.updatedVoxels[step]])
        #     ))) * 1. / stats.updatedVoxels[step])
        #     start += stats.updatedVoxels[step]
        # plt.plot(inconsistentBelief, label=beliefName)
        # plt.plot(inconsistentLogOdds, label=logOddsName)
        # plt.legend(loc='upper right', bbox_to_anchor=(1, .5),
        #            ncol=1, fancybox=True, shadow=True).draggable()



        diffBeliefNeg = []
        diffBeliefPos = []
        diffLogOddsNeg = []
        diffLogOddsPos = []
        avgErrBelief = []
        avgStdBelief = []
        avgErrLogOdds = []
        avgStdLogOdds = []
        maxStdBelief = []
        minStdBelief = []
        maxStdLogOdds = []
        minStdLogOdds = []

        #return
        start = 0
        for step in range(stats.step):
            errLogOdds = np.array(stats.errorCompleteLogOdds[start:start + stats.voxels])
            stdLogOdds = np.array(stats.stdCompleteLogOdds[start:start + stats.voxels])

            errBelief = np.array(stats.errorCompleteBelief[start:start + stats.voxels])
            stdBelief = np.array(stats.stdCompleteBelief[start:start + stats.voxels])
            #stdBelief = correct(stdBelief, errBelief)

            # XXX take only the voxels that were correctly estimated by Log-Odds
            stdBelief = stdBelief[np.abs(errBelief) < 0.3]
            errBelief = errBelief[np.abs(errBelief) < 0.3]
            # print("total voxels:", stats.voxels, "  Correctly predicted:", len(stdBelief))
            #stdBelief = correct(stdBelief, errBelief)
            # print("Mean Belief Std:", np.mean(stdBelief))


            maxStdBelief.append(np.max(stdBelief) if len(stdBelief) > 0 else np.nan)
            minStdBelief.append(np.min(stdBelief) if len(stdBelief) > 0 else np.nan)

            avgErrBelief.append(np.mean(np.abs(errBelief)))
            avgStdBelief.append(np.mean(np.abs(stdBelief)))

            diffBelief = stdBelief - errBelief
            diffBeliefNeg.append(np.abs(np.mean(diffBelief[diffBelief < 0])))
            diffBeliefPos.append(np.abs(np.mean(diffBelief[diffBelief > 0])))


            #stdLogOdds = correct(stdLogOdds, errLogOdds)

            stdLogOdds = stdLogOdds[np.isfinite(errLogOdds)]
            errLogOdds = errLogOdds[np.isfinite(errLogOdds)]
            stdLogOdds = stdLogOdds[np.isfinite(stdLogOdds)]
            errLogOdds = errLogOdds[np.isfinite(stdLogOdds)]
            stdLogOdds = stdLogOdds[np.abs(errLogOdds) < 0.3]
            errLogOdds = errLogOdds[np.abs(errLogOdds) < 0.3]
            maxStdLogOdds.append(np.max(stdLogOdds) if len(stdLogOdds) > 0 else np.nan)
            minStdLogOdds.append(np.min(stdLogOdds) if len(stdLogOdds) > 0 else np.nan)
            #stdLogOdds = correct(stdLogOdds, errLogOdds)

            avgErrLogOdds.append(np.mean(np.abs(errLogOdds)))
            avgStdLogOdds.append(np.mean(np.abs(stdLogOdds)))

            diffLogOdds = stdLogOdds - errLogOdds
            diffLogOddsNeg.append(np.abs(np.mean(diffLogOdds[diffLogOdds < 0])))
            diffLogOddsPos.append(np.abs(np.mean(diffLogOdds[diffLogOdds > 0])))

            start += stats.voxels

        #return
        plt.sca(self.axisPosNegDistanceHybrid)
        plt.cla()
        plt.title("SMAP")
        plt.xlabel("Steps")
        plt.ylabel("Distance Std - Error")

        plt.plot(diffBeliefPos, label="Positive")
        plt.plot(diffBeliefNeg, label="Negative")
        plt.plot(avgErrBelief, label="Avg Abs Error")
        plt.plot(avgStdBelief, label="Avg Std")
        plt.plot(maxStdBelief, label="Max Std")
        plt.plot(minStdBelief, label="Min Std")
        plt.legend(loc='upper right', bbox_to_anchor=(1, .5),
                   ncol=1, fancybox=True, shadow=True).draggable()

        plt.sca(self.axisPosNegDistanceLogOdds)
        plt.cla()
        plt.title("LogOdds")
        plt.xlabel("Steps")
        plt.ylabel("Distance Std - Error")

        plt.plot(diffLogOddsPos, label="Positive")
        plt.plot(diffLogOddsNeg, label="Negative")
        plt.plot(avgErrLogOdds, label="Avg Abs Error")
        plt.plot(avgStdLogOdds, label="Avg Std")
        plt.plot(maxStdLogOdds, label="Max Std")
        plt.plot(minStdLogOdds, label="Min Std")
        plt.legend(loc='upper right', bbox_to_anchor=(1, .5),
                   ncol=1, fancybox=True, shadow=True).draggable()


        ###################################################################################
        # Variance of Std Dev where Prediction Error < 0.3
        ###################################################################################
        varStdBelief = []
        varStdLogOdds = []
        varErrBelief = []
        varErrLogOdds = []
        numCorrectBelief = []
        numCorrectLogOdds = []
        start = 0
        for step in range(stats.step):
            errLogOdds = np.array(stats.errorCompleteLogOdds[start:start + stats.voxels])
            stdLogOdds = np.array(stats.stdCompleteLogOdds[start:start + stats.voxels])
            # stdLogOdds = correct(stdLogOdds, errLogOdds)

            errBelief = np.array(stats.errorCompleteBelief[start:start + stats.voxels])
            stdBelief = np.array(stats.stdCompleteBelief[start:start + stats.voxels])
            # stdBelief = correct(stdBelief, errBelief)

            #XXX take only the voxels that were correctly estimated by Log-Odds
            stdBelief = stdBelief[np.abs(errLogOdds) < 0.3]
            errBelief = errBelief[np.abs(errLogOdds) < 0.3]

            stdLogOdds = stdLogOdds[np.abs(errLogOdds) < 0.3]
            errLogOdds = errLogOdds[np.abs(errLogOdds) < 0.3]

            varStdBelief.append(np.var(stdBelief))
            varErrBelief.append(np.var(errBelief))
            numCorrectBelief.append(len(stdBelief))




            varStdLogOdds.append(np.var(stdLogOdds))
            varErrLogOdds.append(np.var(errLogOdds))
            numCorrectLogOdds.append(len(stdLogOdds))

            start += stats.voxels

        plt.sca(self.axisCorrectStdDevHybrid)
        plt.cla()
        plt.title("Variance of Std Dev where Log Prediction Error < 0.3")
        plt.xlabel("Steps")
        plt.ylabel("Variance")

        plt.plot(varStdBelief, label="Var Std SMAP")
        plt.plot(varStdLogOdds, label="Var Std Log-Odds")
        plt.plot(varErrBelief, label="Var Error SMAP")
        plt.plot(varErrLogOdds, label="Var Error Log-Odds")
        plt.legend(loc='upper right', bbox_to_anchor=(1, .5),
                   ncol=1, fancybox=True, shadow=True).draggable()

        plt.sca(self.axisCorrectStdDevLogOdds)
        plt.cla()
        plt.title("Number of Updated Voxels where Prediction Error < 0.3")
        plt.xlabel("Steps")
        plt.ylabel("Number")

        plt.plot(numCorrectBelief, label=beliefName)
        plt.plot(numCorrectLogOdds, label=logOddsName, )
        plt.legend(loc='upper right', bbox_to_anchor=(1, .5),
                   ncol=1, fancybox=True, shadow=True).draggable()


        plt.sca(self.axisCorrelation)
        plt.cla()
        plt.title("Absolute Distance between Absolute Error and Mapping Std Dev")
        plt.xlabel("Steps")
        plt.ylabel("Absolute Distance")

        return # TODO remove

        def my_pearson(x, y):
            r, _ = pearsonr(x, y)
            return r
        pccBelief = []
        pccLogOdds = []
        start = 0
        for step in range(stats.step):
            #print("Computing PCC...")
            #print("Updated Voxels: %i" % stats.updatedVoxels[step])
            pcc = my_pearson(
                np.abs(np.array(stats.errorCompleteUpdatedBelief[start:start + stats.updatedVoxels[step]])),
                stats.stdCompleteUpdatedBelief[start:start + stats.updatedVoxels[step]])
            pccBelief.append(pcc)
            pcc = my_pearson(
                np.abs(np.array(stats.errorCompleteUpdatedLogOdds[start:start + stats.updatedVoxels[step]])),
                stats.stdCompleteUpdatedLogOdds[start:start + stats.updatedVoxels[step]])
            pccLogOdds.append(pcc)
            start += stats.updatedVoxels[step]
        # TODO reactivate Python's PCC computation?
        # plt.plot(pccBelief, label="PY " + beliefName)
        # plt.plot(pccLogOdds, label="PY " + logOddsName)
        plt.plot(np.array(stats.stdErrorCorrelationBelief), label=beliefName)
        stats.stdErrorCorrelationLogOdds = np.array(stats.stdErrorCorrelationLogOdds)
        last_finite = 1.
        for i in range(len(stats.stdErrorCorrelationLogOdds)):
            if np.isfinite(stats.stdErrorCorrelationLogOdds[i]):
                last_finite = stats.stdErrorCorrelationLogOdds[i]
            else:
                stats.stdErrorCorrelationLogOdds[i] = last_finite

        mask = np.isfinite(stats.stdErrorCorrelationLogOdds)
        #stats.stdErrorCorrelationLogOdds[~mask] = 0
        plt.plot(np.array(stats.stdErrorCorrelationLogOdds), label=logOddsName)
        smoothedCorrelationBelief = np.convolve(stats.stdErrorCorrelationBelief, np.ones(100) / 100)
        smoothedCorrelationLogOdds = np.convolve(stats.stdErrorCorrelationLogOdds, np.ones(100) / 100)
        mask = np.isfinite(smoothedCorrelationLogOdds)
        xs = np.array(list(range(len(stats.stdErrorCorrelationLogOdds)+100)))
        plt.plot(smoothedCorrelationBelief, label="100 Point Moving Avg SMAP")
        plt.plot(xs[mask], smoothedCorrelationLogOdds[mask], label="100 Point Moving Avg Log Odds")
        plt.legend(loc='upper right', bbox_to_anchor=(1, .5),
                   ncol=1, fancybox=True, shadow=True).draggable()
        self.axisCorrelation.set_xlim([0, len(stats.stdErrorCorrelationBelief)])
        #self.axisCorrelation.set_ylim([0, 1])



        stdLastBelief = np.array(stats.stdCompleteUpdatedBelief[-stats.updatedVoxels[-1]:])
        #print "stats.stdCompleteUpdatedBelief", stats.stdCompleteUpdatedBelief
        stdLastLogOdds = np.array(stats.stdCompleteUpdatedLogOdds[-stats.updatedVoxels[-1]:])
        plt.sca(self.axisLastHybrid)
        plt.cla()
        plt.title("SMAP")

        plt.plot(stats.errorCompleteUpdatedBelief[-stats.updatedVoxels[-1]:])
        plt.plot(-stdLastBelief * self.mag, 'r')
        plt.plot(stdLastBelief * self.mag, 'r')

        # add scatter of inconsistencies
        # lb = np.array(-stdLastBelief * self.mag)
        # ub = np.array(stdLastBelief * self.mag)
        # err = np.array(stats.errorCompleteUpdatedBelief[-stats.updatedVoxels[-1]:])
        # inconsistencies = err[(err > ub) | (err < lb)]
        # indices = np.array(list(range(stats.updatedVoxels[-1])))
        # indices = indices[(err > ub) | (err < lb)]
        # plt.scatter(indices, inconsistencies, c='g', edgecolor='g')

        plt.sca(self.axisLast_logOdds)
        plt.cla()
        plt.title("Log Odds")
        plt.plot(stats.errorCompleteUpdatedLogOdds[-stats.updatedVoxels[-1]:])
        plt.plot(-stdLastLogOdds * self.mag, 'r')
        plt.plot(stdLastLogOdds * self.mag, 'r')



        plt.sca(self.axisLastCorrectHybrid)
        plt.cla()
        plt.title("SMAP")
        lastError = np.array(stats.errorCompleteUpdatedBelief[-stats.updatedVoxels[-1]:])
        stdLastCorrectBelief = stdLastBelief[lastError < 0.3]
        lastError = lastError[lastError < 0.3]
        plt.plot(lastError)
        plt.plot(stdLastCorrectBelief, 'r')
        self.axisLastCorrectHybrid.set_xlim([0, len(lastError)])

        plt.sca(self.axisLastCorrect_logOdds)
        plt.cla()
        plt.title("Log Odds")
        lastError = np.array(stats.errorCompleteUpdatedLogOdds[-stats.updatedVoxels[-1]:])
        stdLastCorrectLogOdds = correct(stdLastLogOdds[lastError < 0.3], lastError[lastError < 0.3])
        lastError = lastError[lastError < 0.3]
        plt.plot(lastError)
        plt.plot(stdLastCorrectLogOdds, 'r')
        self.axisLastCorrect_logOdds.set_xlim([0, len(lastError)])



        # TODO remove
        # return
        #if len(stats.errorCompleteUpdatedBelief) == 0 or len(stats.errorCompleteUpdatedLogOdds) == 0:
        #    return

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


        plt.plot(np.abs(stats.errorCompleteUpdatedBelief[-stats.updatedVoxels[-1]:]))
        #plt.plot(-stdLastBelief * self.mag, 'r')
        plt.plot(stdLastBelief * self.mag, 'r')

        # add scatter of inconsistencies
        # lb = np.array(-stdLastBelief * self.mag)
        # ub = np.array(stdLastBelief * self.mag)
        # err = np.array(stats.errorCompleteUpdatedBelief[-stats.updatedVoxels[-1]:])
        # inconsistencies = err[(err > ub) | (err < lb)]
        # indices = np.array(list(range(stats.updatedVoxels[-1])))
        # indices = indices[(err > ub) | (err < lb)]
        # plt.scatter(indices, inconsistencies, c='g', edgecolor='g')

        plt.sca(self.axisLast_logOdds)
        plt.cla()
        plt.title("Log Odds")
        self.axisLast_logOdds.set_xlim([0, stats.updatedVoxels[-1]])
        plt.plot(np.abs(stats.errorCompleteUpdatedLogOdds[-stats.updatedVoxels[-1]:]))
        #plt.plot(-stdLastLogOdds * self.mag, 'r')
        plt.plot(stdLastLogOdds * self.mag, 'r')

        # add scatter of inconsistencies
        # lb = np.array(-stdLastLogOdds * self.mag)
        # ub = np.array(stdLastLogOdds * self.mag)
        # err = np.array(stats.errorCompleteUpdatedLogOdds[-stats.updatedVoxels[-1]:])
        # inconsistencies = err[(err > ub) | (err < lb)]
        # indices = np.array(list(range(stats.updatedVoxels[-1])))
        # indices = indices[(err > ub) | (err < lb)]
        # plt.scatter(indices, inconsistencies, c='g', edgecolor='g')

        plt.pause(1e-6)


        # corrected std dev compared to actual error

        # def correct(std, error):
        #     ms = np.mean(std)
        #     me = np.mean(error)
        #     return std * (ms/me)

        # def correct(std, error):
        #     X = np.array(list(range(len(std))))
        #     par = np.polyfit(X, std, 1, full=True)
        #     print "std params: ", par[0]
        #     sSlope = par[0][0]
        #     sIntercept = par[0][1]
        #     par = np.polyfit(X, error, 1, full=True)
        #     print "err params: ", par[0]
        #     eSlope = par[0][0]
        #     eIntercept = par[0][1]
        #
        #     newStd = (std - (sIntercept+eIntercept)*(sSlope/eSlope)) * (eSlope/sSlope)
        #     par = np.polyfit(X, newStd, 1, full=True)
        #     print "new params: ", par[0]
        #
        #     return newStd



        stdLastBelief = np.abs(np.array(stats.stdCompleteUpdatedBelief[-stats.updatedVoxels[-1]:]))
        stdLastLogOdds = np.abs(np.array(stats.stdCompleteUpdatedLogOdds[-stats.updatedVoxels[-1]:]))
        plt.sca(self.axisLastCorrectedHybrid)
        plt.cla()
        plt.title("SMAP")
        self.axisLastCorrectedHybrid.set_xlim([0, stats.updatedVoxels[-1]])
        self.axisLastCorrectedHybrid.set_ylim([0, 1])

        errorBelief = np.abs(stats.errorCompleteUpdatedBelief[-stats.updatedVoxels[-1]:])
        plt.plot(errorBelief)
        correctedBeliefStd = correct(stdLastBelief, errorBelief)
        plt.plot(correctedBeliefStd, 'r')

        # add scatter of inconsistencies
        # lb = np.array(-stdLastBelief * self.mag)
        # ub = np.array(stdLastBelief * self.mag)
        # err = np.array(stats.errorCompleteUpdatedBelief[-stats.updatedVoxels[-1]:])
        # inconsistencies = err[(err > ub) | (err < lb)]
        # indices = np.array(list(range(stats.updatedVoxels[-1])))
        # indices = indices[(err > ub) | (err < lb)]
        # plt.scatter(indices, inconsistencies, c='g', edgecolor='g')

        plt.sca(self.axisLastCorrected_logOdds)
        plt.cla()
        plt.title("Log Odds")
        self.axisLastCorrected_logOdds.set_xlim([0, stats.updatedVoxels[-1]])
        self.axisLastCorrected_logOdds.set_ylim([0, 1])

        errorLogOdds = np.abs(stats.errorCompleteUpdatedLogOdds[-stats.updatedVoxels[-1]:])
        plt.plot(errorLogOdds)
        correctedLogOddsStd = correct(stdLastLogOdds, errorLogOdds)
        plt.plot(correctedLogOddsStd, 'r')

        # add scatter of inconsistencies
        # lb = np.array(-stdLastLogOdds * self.mag)
        # ub = np.array(stdLastLogOdds * self.mag)
        # err = np.array(stats.errorCompleteUpdatedLogOdds[-stats.updatedVoxels[-1]:])
        # inconsistencies = err[(err > ub) | (err < lb)]
        # indices = np.array(list(range(stats.updatedVoxels[-1])))
        # indices = indices[(err > ub) | (err < lb)]
        # plt.scatter(indices, inconsistencies, c='g', edgecolor='g')

        plt.pause(1e-6)

        stdFirstBelief = np.abs(np.array(stats.stdCompleteUpdatedBelief[:stats.updatedVoxels[0]]))
        stdFirstLogOdds = np.abs(np.array(stats.stdCompleteUpdatedLogOdds[:stats.updatedVoxels[0]]))
        plt.sca(self.axisFirstCorrectedHybrid)
        plt.cla()
        plt.title("SMAP")
        self.axisFirstCorrectedHybrid.set_xlim([0, stats.updatedVoxels[0]])
        self.axisFirstCorrectedHybrid.set_ylim([0, 1])

        errorBelief = np.abs(stats.errorCompleteUpdatedBelief[:stats.updatedVoxels[0]])
        plt.plot(errorBelief)
        correctedBeliefStd = correct(stdFirstBelief, errorBelief)
        plt.plot(correctedBeliefStd, 'r')

        # add scatter of inconsistencies
        # lb = np.array(-stdFirstBelief * self.mag)
        # ub = np.array(stdFirstBelief * self.mag)
        # err = np.array(stats.errorCompleteUpdatedBelief[-stats.updatedVoxels[-1]:])
        # inconsistencies = err[(err > ub) | (err < lb)]
        # indices = np.array(list(range(stats.updatedVoxels[-1])))
        # indices = indices[(err > ub) | (err < lb)]
        # plt.scatter(indices, inconsistencies, c='g', edgecolor='g')

        plt.sca(self.axisFirstCorrected_logOdds)
        plt.cla()
        plt.title("Log Odds")
        self.axisFirstCorrected_logOdds.set_xlim([0, stats.updatedVoxels[0]])
        self.axisFirstCorrected_logOdds.set_ylim([0, 1])

        errorLogOdds = np.abs(stats.errorCompleteUpdatedLogOdds[:stats.updatedVoxels[0]])
        plt.plot(errorLogOdds)
        correctedLogOddsStd = correct(stdFirstLogOdds, errorLogOdds)
        plt.plot(correctedLogOddsStd, 'r')

        # add scatter of inconsistencies
        # lb = np.array(-stdFirstLogOdds * self.mag)
        # ub = np.array(stdFirstLogOdds * self.mag)
        # err = np.array(stats.errorCompleteUpdatedLogOdds[-stats.updatedVoxels[-1]:])
        # inconsistencies = err[(err > ub) | (err < lb)]
        # indices = np.array(list(range(stats.updatedVoxels[-1])))
        # indices = indices[(err > ub) | (err < lb)]
        # plt.scatter(indices, inconsistencies, c='g', edgecolor='g')

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
        plt.plot(np.abs(stats.errorCompleteUpdatedBelief[:stats.updatedVoxels[0]]))
        self.axisHybrid.set_xlim([0, stats.updatedVoxels[0]])
        #plt.plot(-stdLastBelief * self.mag, 'r')
        plt.plot(stdLastBelief * self.mag, 'r')

        # add scatter of inconsistencies
        # lb = np.array(-stdLastBelief * self.mag)
        # ub = np.array(stdLastBelief * self.mag)
        # err = np.array(stats.errorCompleteUpdatedBelief[:stats.updatedVoxels[0]])
        # inconsistencies = err[(err > ub) | (err < lb)]
        # indices = np.array(list(range(stats.updatedVoxels[0])))
        # indices = indices[(err > ub) | (err < lb)]
        # plt.scatter(indices, inconsistencies, c='g', edgecolor='g')


        plt.sca(self.axis_logOdds)
        plt.cla()
        plt.title("Log Odds")
        plt.plot(np.abs(stats.errorCompleteUpdatedLogOdds[:stats.updatedVoxels[0]]))
        self.axis_logOdds.set_xlim([0, stats.updatedVoxels[0]])
        #plt.plot(-stdLastLogOdds * self.mag, 'r')
        plt.plot(stdLastLogOdds * self.mag, 'r')

        # add scatter of inconsistencies
        # lb = np.array(-stdLastLogOdds * self.mag)
        # ub = np.array(stdLastLogOdds * self.mag)
        # err = np.array(stats.errorCompleteUpdatedLogOdds[:stats.updatedVoxels[0]])
        # inconsistencies = err[(err > ub) | (err < lb)]
        # indices = np.array(list(range(stats.updatedVoxels[0])))
        # indices = indices[(err > ub) | (err < lb)]
        # plt.scatter(indices, inconsistencies, c='g', edgecolor='g')

        plt.pause(1e-6)

        # plt.sca(self.axisSparseHybrid)
        # plt.cla()
        # plt.title("SMAP")
        # plt.plot(errorHybrid[::100])
        # plt.plot(-stdVectorHybrid[::100] * self.mag, 'r')pyt
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
            for errorBelief in stats.errorCompleteUpdatedBelief[start:start + stats.updatedVoxels[step]]:
                errorBelief = abs(errorBelief)
                # if abs(error - 0.5) < 1e-2:
                #     continue
                # print error
                bin_number = int(round(bins * (1 - (errorBelief - miny) / (maxy - miny))))
                if 0 <= bin_number < bins:
                    b[bin_number] += 1
                    # if bin_number != 50:
                    #     print bin_number
                sum += errorBelief
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
            for errorBelief in stats.errorCompleteUpdatedLogOdds[start:start + stats.updatedVoxels[step]]:
                errorBelief = abs(errorBelief)
                # if abs(error - 0.5) < 1e-2:
                #     continue
                # print error
                bin_number = int(round(bins * (1 - (errorBelief - miny) / (maxy - miny))))
                if 0 <= bin_number < bins:
                    b[bin_number] += 1
                sum += errorBelief
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
