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

# std deviation per voxel at current step
float64[] stdBelief
float64[] stdLogOdds

# std deviation for each voxel over all steps (size is step*voxels)
float64[] stdCompleteBelief
float64[] stdCompleteLogOdds

# noise std deviation for observation simulation,
# and SMAP's likelihoodGivenCause computation
float64 noiseStd

# ISM configuration
float64 ismIncrement
float64 ismRampSize
float64 ismTopSize
float64 ismRampSlope
