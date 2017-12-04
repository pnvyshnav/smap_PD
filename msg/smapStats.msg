Header header

# current step number
uint32 step

# number of total steps
uint32 maxStep

# number of voxels
uint64 voxels

# particle values
float32[] particles
float32[] beliefMeans
float32[] beliefStds
bool[] trueMeans

# computation time of single step
float64[] timeBelief
float64[] timeLogOdds
float64[] timeGP

# number of measurents per step
int32[] measurements

# ray length statistics over the measurements per step
float64[] minRayLength
float64[] maxRayLength
float64[] avgRayLength

# error for each voxel at current step
float64[] errorBelief
float64[] errorLogOdds
float64[] errorGP

# L1-error evolution, i.e. sum of errors for all steps
float64[] errorEvolutionBelief
float64[] errorEvolutionLogOdds
float64[] errorEvolutionGP

# L1-error evolution of updated voxels at last step,
# i.e. sum of errors for all steps
float64[] errorEvolutionUpdatedBelief
float64[] errorEvolutionUpdatedLogOdds

# abs errors and  std at final step of all voxels that were updated
# at SMAP and Log Odds over the whole runs
float64[] errorFinalUpdatedBelief
float64[] errorFinalUpdatedLogOdds
float64[] stdFinalUpdatedBelief
float64[] stdFinalUpdatedLogOdds

# std evolution, i.e. sum of stds for all steps
float64[] stdEvolutionBelief
float64[] stdEvolutionLogOdds
float64[] stdEvolutionGP

# error for each voxel over all steps (size is step*voxels)
float64[] errorCompleteBelief
float64[] errorCompleteLogOdds

# number of updated voxels at each step
uint64[] updatedVoxels

# positions of updated voxels (size is sum of updatedVoxels)
float64[] updatedVoxelsX
float64[] updatedVoxelsY
float64[] updatedVoxelsZ

# error for each updated voxel over all steps (size is sum of updatedVoxels)
float64[] errorCompleteUpdatedBelief
float64[] errorCompleteUpdatedLogOdds

# std deviation per voxel at current step
float64[] stdBelief
float64[] stdLogOdds
float64[] stdGP

# std deviation for each voxel over all steps (size is step*voxels)
float64[] stdCompleteBelief
float64[] stdCompleteLogOdds

# std deviation for each updated voxel over all steps (size is sum of updatedVoxels)
float64[] stdCompleteUpdatedBelief
float64[] stdCompleteUpdatedLogOdds

# evolution of the correlation between std dev and error of updated voxels
float64[] stdErrorCorrelationBelief
float64[] stdErrorCorrelationLogOdds

# evolution of the correlation between std dev and error of *all* voxels
float64[] stdErrorCorrelationGP

# noise std deviation for observation simulation,
# and SMAP's likelihoodGivenCause computation
float64 noiseStd

# ISM configuration
float64 ismIncrement
float64 ismRampSize
float64 ismTopSize
float64 ismRampSlope

# GP configuration
float64 gpParameter1
float64 gpParameter2
float64 gpParameter3
string kernel

#################################################################
#         Scenario of Trajectory Evaluation (Planning)          #
#################################################################

uint32 trajectoryId

# Number of voxels per trajectory
uint32 trajectoryVoxels
float64[] trajectoryVoxelX
float64[] trajectoryVoxelY

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

# trajectory control points 2d positions
float64[] trajectoryControlX
float64[] trajectoryControlY

# Future predictions
# reachability of voxels within time span
float64[] trajectoryFutureReachability
# number of future voxels to be considered
uint32[] trajectoryFutureVoxels

# Evolution of occupancies of future voxels (size is maxStep*sum(trajectoryFutureVoxels))
float64[] trajectoryFutureOccupanciesBelief
#float64[] trajectoryFutureOccupanciesLogOdds
# Evolution of std devs of future voxels (size is maxStep*sum(trajectoryFutureVoxels))
float64[] trajectoryFutureStdDevsBelief
#float64[] trajectoryFutureStdDevsLogOdds