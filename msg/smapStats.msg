Header header

# current step number
uint32 step

# number of total steps
uint32 maxStep

# number of voxels
uint64 voxels

# error for each voxel at current step
float64[] errorBelief
float64[] errorLogOdds

# L1-error evolution, i.e. sum of errors for all steps
float64[] errorEvolutionBelief
float64[] errorEvolutionLogOdds

# error for each voxel over all steps (size is step*voxels)
float64[] errorCompleteBelief
float64[] errorCompleteLogOdds

# number of updated voxels at each step
uint64[] updatedVoxels

# error for each updated voxel over all steps (size is sum of updatedVoxels)
float64[] errorCompleteUpdatedBelief
float64[] errorCompleteUpdatedLogOdds

# std deviation per voxel at current step
float64[] stdBelief
float64[] stdLogOdds

# std deviation for each voxel over all steps (size is step*voxels)
float64[] stdCompleteBelief
float64[] stdCompleteLogOdds

# std deviation for each updated voxel over all steps (size is sum of updatedVoxels)
float64[] stdCompleteUpdatedBelief
float64[] stdCompleteUpdatedLogOdds

# evolution of the correlation between std dev and error of updated voxels
float64[] stdErrorCorrelationBelief
float64[] stdErrorCorrelationLogOdds

# noise std deviation for observation simulation,
# and SMAP's likelihoodGivenCause computation
float64 noiseStd

# ISM configuration
float64 ismIncrement
float64 ismRampSize
float64 ismTopSize
float64 ismRampSlope

#################################################################
#         Scenario of Trajectory Evaluation (Planning)          #
#################################################################

uint32 trajectoryId

# Number of voxels per trajectory
uint32 trajectoryVoxels

# Evolution of occupancy predictions along each trajectory (size is maxStep*trajectoryVoxels)
float64[] trajectoryOccupanciesBelief
float64[] trajectoryOccupanciesLogOdds
# Evolution of std dev predictions along each trajectory (size is maxStep*trajectoryVoxels)
float64[] trajectoryStdDevsBelief
float64[] trajectoryStdDevsLogOdds

# trajectory 2d position, yaw angle (theta) at each step
float64[] trajectoryX
float64[] trajectoryY
float64[] trajectoryT
float64[] trajectoryTime

# future measurements
float64[] trajectoryFutureReachability
float64[] trajectoryFutureSafety
uint32[] trajectoryFutureSteps
uint32[] trajectoryFutureVoxels