import matplotlib.pyplot as plt
import numpy as np
from os import listdir
from os.path import isfile, join
from paramList import *
from mapTrue import *

plt.close('all')

directoryName = "C:/Users/Ali/Desktop/SMAP_python_Code/results/2016-09-11-19-27-15_SixDifferentStd/std"
onlyfiles = [f for f in listdir(directoryName) if isfile(join(directoryName, f))]

for f in onlyfiles:
    file = directoryName + '/' + f
    data = np.load(file)
    meanVectorHybrid = data[0]
    meanVectorLogOdds = data[1]
    trueDensities = data[2]
    stdVectorHybrid = data[3]
    stdVectorLogOdds = data[4]
    ErrorOverFullMapHybrid = data[5]
    ErrorOverFullMapLogOdds = data[6]

    errorHybrid = meanVectorHybrid - trueDensities
    errorLogOdds = meanVectorLogOdds - trueDensities

    tmp = plt.plot(ErrorOverFullMapHybrid, '-')
    tmp2 = plt.plot(ErrorOverFullMapLogOdds, '--')
    tmp2[0].set_color(tmp[0].get_color())
    plt.pause(1e-6)

plt.xlabel('Time-step')
plt.ylabel('Map error')
plt.savefig('savedFigures/differentStds', bbox_inches='tight', pad_inches=0.1)


####################################################
####################################################
plt.figure('bar')
plt.figure('inv')
directoryName = "C:/Users/Ali/Desktop/SMAP_python_Code/results/2016-09-12-13-44-00"
onlyfiles = [f for f in listdir(directoryName) if isfile(join(directoryName, f))]
ctr = 1
for f in onlyfiles:
    file = directoryName + '/' + f
    data = np.load(file)
    meanVectorHybrid = data[0]
    meanVectorLogOdds = data[1]
    trueDensities = data[2]
    stdVectorHybrid = data[3]
    stdVectorLogOdds = data[4]
    ErrorOverFullMapHybrid = data[5]
    ErrorOverFullMapLogOdds = data[6]

    errorHybrid = meanVectorHybrid - trueDensities
    errorLogOdds = meanVectorLogOdds - trueDensities

    #tmp = plt.plot(ErrorOverFullMapHybrid, '-')
    tmp2 = plt.plot(ErrorOverFullMapLogOdds, '--')
    # tmp2[0].set_color(tmp[0].get_color())
    plt.pause(1e-6)

    consistencyGapLogOdds = abs(errorLogOdds) - Par.varianceMagnification * stdVectorLogOdds
    ind_inconsistentLogOdds = np.nonzero(consistencyGapLogOdds > 0)
    sumOfPositiveGapLogOdds = sum(consistencyGapLogOdds[ind_inconsistentLogOdds])
    numInconsisLogOdds = len(ind_inconsistentLogOdds[0])
    normalizedLogOdds = sumOfPositiveGapLogOdds / numInconsisLogOdds

    plt.plot(300, sumOfPositiveGapLogOdds, 's', color = tmp2[0].get_color())

    plt.figure('bar')
    plt.bar(ctr, ErrorOverFullMapLogOdds[-1], yerr = sumOfPositiveGapLogOdds/2,
            error_kw=dict(ecolor='black', lw=2, capsize=5, capthick=2))
    ctr = ctr + 1
    plt.pause(1e-6)
    plt.figure('inv')

file = "C:/Users/Ali/Desktop/SMAP_python_Code/results/2016-09-11-19-27-15_SixDifferentStd/std/std_0.2.npy"
data = np.load(file)
meanVectorHybrid = data[0]
meanVectorLogOdds = data[1]
trueDensities = data[2]
stdVectorHybrid = data[3]
stdVectorLogOdds = data[4]
ErrorOverFullMapHybrid = data[5]
ErrorOverFullMapLogOdds = data[6]

errorHybrid = meanVectorHybrid - trueDensities
errorLogOdds = meanVectorLogOdds - trueDensities

tmp = plt.plot(ErrorOverFullMapHybrid, '-')
plt.pause(1e-6)

consistencyGapHybrid = abs(errorHybrid) - Par.varianceMagnification * stdVectorHybrid
ind_inconsistentHybrid = np.nonzero(consistencyGapHybrid > 0)
sumOfPositiveGapHybrid = sum(consistencyGapHybrid[ind_inconsistentHybrid])
numInconsisHybrid = len(ind_inconsistentHybrid[0])
normalizedHybrid = sumOfPositiveGapHybrid / numInconsisHybrid
normalizedLogOdds = sumOfPositiveGapLogOdds / numInconsisLogOdds
plt.plot(300, sumOfPositiveGapHybrid, 'o', color = tmp2[0].get_color())

plt.figure('bar')
plt.bar(ctr, ErrorOverFullMapHybrid[-1], yerr= sumOfPositiveGapHybrid/2, color='r',
            error_kw=dict(ecolor='black', lw=2, capsize=5, capthick=2))
ctr = ctr + 1
plt.pause(1e-6)
plt.figure('bar')
plt.ylim([100,260])
plt.xlabel('Different ISM parameters')
plt.ylabel('Map error/inconsistency')
plt.savefig('savedFigures/barChart', bbox_inches='tight', pad_inches=0.1)


plt.figure('inv')
plt.xlabel('Time-step')
plt.ylabel('Map error')
plt.savefig('savedFigures/differentInvs', bbox_inches='tight', pad_inches=0.1)

####################################################
####################################################
plt.figure('imp')
plt.figure('consis')
directoryName = "C:/Users/Ali/Desktop/SMAP_python_Code/results/2016-09-12-15-36-39/MonteCarlo"
onlyfiles = [f for f in listdir(directoryName) if isfile(join(directoryName, f))]

sumOfErrorOverRuns_Hybrid = 0
sumOfErrorOverRuns_LogOdds = 0

for f,file_ctr in zip(onlyfiles,range(len(onlyfiles))):
    file = directoryName + '/' + f
    data = np.load(file)
    meanVectorHybrid = data[0]
    meanVectorLogOdds = data[1]
    trueDensities = data[2]
    stdVectorHybrid = data[3]
    stdVectorLogOdds = data[4]
    ErrorOverFullMapHybrid = data[5]
    ErrorOverFullMapLogOdds = data[6]

    improvement = np.array(ErrorOverFullMapLogOdds) - np.array(ErrorOverFullMapHybrid)
    improvementPercentage = 100*(np.array(ErrorOverFullMapLogOdds) - np.array(ErrorOverFullMapHybrid))/np.array(ErrorOverFullMapHybrid)

    plt.figure('imp')
    tmp = plt.plot(improvementPercentage, '-')
    plt.pause(1e-6)

    sumOfErrorOverRuns_Hybrid = np.array(sumOfErrorOverRuns_Hybrid) + np.array(ErrorOverFullMapHybrid)
    sumOfErrorOverRuns_LogOdds = np.array(sumOfErrorOverRuns_LogOdds) + np.array(ErrorOverFullMapLogOdds)

    errorHybrid = meanVectorHybrid - trueDensities
    errorLogOdds = meanVectorLogOdds - trueDensities
    thresh = Par.largeErrorThreshold
    ind_large = np.nonzero((abs(errorHybrid) > thresh) | (abs(errorLogOdds) > thresh))
    largeErrorsHybrid = errorHybrid[ind_large]
    largeErrorsLogodds = errorLogOdds[ind_large]
    largeErrVarianceHybrid = stdVectorHybrid[ind_large]
    largeErrVarianceLogodds = stdVectorLogOdds[ind_large]

    consistencyGapHybrid = abs(errorHybrid) - Par.varianceMagnification*stdVectorHybrid
    consistencyGapLogOdds = abs(errorLogOdds) - Par.varianceMagnification * stdVectorLogOdds
    ind_inconsistentHybrid = np.nonzero(consistencyGapHybrid > 0)
    ind_inconsistentLogOdds = np.nonzero(consistencyGapLogOdds > 0)

    sumOfPositiveGapHybrid = sum(consistencyGapHybrid[ind_inconsistentHybrid])
    sumOfPositiveGapLogOdds = sum(consistencyGapLogOdds[ind_inconsistentLogOdds])
    numInconsisHybrid = len(ind_inconsistentHybrid[0])
    numInconsisLogOdds = len(ind_inconsistentLogOdds[0])
    normalizedHybrid = sumOfPositiveGapHybrid/numInconsisHybrid
    normalizedLogOdds = sumOfPositiveGapLogOdds/numInconsisLogOdds

    plt.figure('consis')
    plt.plot(file_ctr, sumOfPositiveGapHybrid,'bo')
    plt.plot(file_ctr, sumOfPositiveGapLogOdds,'rs')
    # plt.plot(file_ctr, normalizedHybrid, 'bo')
    # plt.plot(file_ctr, normalizedLogOdds, 'b*')
    # plt.plot(file_ctr, numInconsisHybrid, 'go')
    # plt.plot(file_ctr, numInconsisLogOdds, 'g*')

    plt.pause(1e-6)

plt.figure('imp')
plt.xlabel('run')
plt.ylabel('Error Improvement')
plt.savefig('savedFigures/performance', bbox_inches='tight', pad_inches=0.1)

plt.figure('consis')
plt.xlabel('run')
plt.ylabel('Filter Inconsistency')
plt.savefig('savedFigures/inconsistency', bbox_inches='tight', pad_inches=0.1)



###############################################
f = onlyfiles[11]
file = directoryName + '/' + f
data = np.load(file)
meanVectorHybrid = data[0]
meanVectorLogOdds = data[1]
trueDensities = data[2]
stdVectorHybrid = data[3]
stdVectorLogOdds = data[4]
ErrorOverFullMapHybrid = data[5]
ErrorOverFullMapLogOdds = data[6]

plt.figure()
mapT = TrueMap()
for vox,meanT in zip(mapT.voxels,trueDensities):
    vox.density = meanT
mapT.draw()
plt.savefig('savedFigures/MapTrue', bbox_inches='tight', pad_inches=0.1)

plt.figure()
mapH = TrueMap()
for vox,meanH in zip(mapH.voxels,meanVectorHybrid):
    vox.density = meanH
mapH.draw()
plt.savefig('savedFigures/MapHybrid', bbox_inches='tight', pad_inches=0.1)

plt.figure()
mapL = TrueMap()
for vox,meanL in zip(mapL.voxels,meanVectorLogOdds):
    vox.density = meanL
mapL.draw()
plt.savefig('savedFigures/MapLogOdds', bbox_inches='tight', pad_inches=0.1)

plt.figure()
plt.plot(ErrorOverFullMapHybrid, 'bo')
plt.plot(ErrorOverFullMapLogOdds, 'r*')
plt.xlabel('Time-step')
plt.ylabel('Error')
plt.savefig('savedFigures/error', bbox_inches='tight', pad_inches=0.1)


errorHybrid = meanVectorHybrid - trueDensities
errorLogOdds = meanVectorLogOdds - trueDensities
thresh = Par.largeErrorThreshold
ind_large = np.nonzero((abs(errorHybrid)>thresh) | (abs(errorLogOdds)>thresh))
largeErrorsHybrid = errorHybrid[ind_large]
largeErrorsLogodds = errorLogOdds[ind_large]
largeErrVarianceHybrid = stdVectorHybrid[ind_large]
largeErrVarianceLogodds = stdVectorLogOdds[ind_large]

fig_zoomedInconsistency = plt.figure('zoomedOnInconsistencies')
mag = Par.varianceMagnification

axisHybridLargeErrors = fig_zoomedInconsistency.add_subplot(211)
plt.plot(largeErrorsHybrid)
plt.plot(-largeErrVarianceHybrid * mag, 'r')
plt.plot(largeErrVarianceHybrid * mag, 'r')
plt.ylabel('Error mean/variance')

axisLogoddsLargeErrors = fig_zoomedInconsistency.add_subplot(212)
plt.plot(largeErrorsLogodds)
plt.plot(-largeErrVarianceLogodds * mag, 'r')
plt.plot(largeErrVarianceLogodds * mag, 'r')
plt.xlabel('voxel')
plt.ylabel('Error mean/variance')
plt.savefig('savedFigures/MeanVarianceLogOdds', bbox_inches='tight', pad_inches=0.1)

plt.show()