Header header

uint32 step

# error for each voxel
float64[] errorBelief
float64[] errorLogOdds

# L1-error evolution
float64[] errorEvolutionBelief
float64[] errorEvolutionLogOdds

# std deviation per voxel
float64[] stdBelief
float64[] stdLogOdds

# belief configuration
float64 noiseStd

# ISM configuration
float64 ismIncrement
float64 ismRampSize
float64 ismTopSize
float64 ismRampSlope
