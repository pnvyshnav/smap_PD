#include "../include/StereoCameraSensor.h"
#include "../include/BeliefMap.h"


StereoCameraSensor::StereoCameraSensor(Parameters::Vec3Type &position, Parameters::Vec3Type &orientation)
        : Sensor(position, orientation) {}

Observation StereoCameraSensor::observe(TrueMap &trueMap) const
{
    std::vector<Measurement> measurements;
    for (auto &sensor : _pixelSensors)
        measurements.push_back(sensor.observe(trueMap));
    return Observation(measurements);
}

std::vector<PixelSensor> StereoCameraSensor::pixels() const
{
    return _pixelSensors;
}

StereoCameraSensor::InverseCauseModel StereoCameraSensor::computeInverseCauseModel(Measurement measurement, BeliefMap &beliefMap)
{
    InverseCauseModel icm;
    octomap::KeyRay ray;
    beliefMap.computeRayKeys(position(), orientation() * Parameters::sensorRange, ray);
    std::valarray<Parameters::NumType> bouncingProbabilities = beliefMap.bouncingProbabilitiesOnRay(ray);
    std::valarray<Parameters::NumType> reachingProbabilities = beliefMap.reachingProbabilitiesOnRay(ray, bouncingProbabilities);
    std::valarray<Parameters::NumType> prior = bouncingProbabilities * reachingProbabilities;


//    cause_voxels_geometry = [mapBelief.grid.voxelsGeom[i] for i in cause_global_ids]
//    causeProb_posterior_onRay = []
//    for cause_geom, prior in zip(cause_voxels_geometry, causeProbabilities_prior):
//    likelihood = self.likelihoodGivenCause(measurement, sensorPosition, cause_geom)
//    causeProb_posterior_onRay.append(likelihood * prior)
//    causeProbabilities_posterior = {'onRay': causeProb_posterior_onRay}
//
//    bouncingProbFromInfinity = 1.
//    ReachingProbFromInfinity = (1 - bouncingProbabilities[-1]) * reachingProbabilities[-1]
//    causeProbFromInfinity_prior = ReachingProbFromInfinity * bouncingProbFromInfinity
//    measurementLikelihoodGivenInfinity = self.likelihoodGivenCause(measurement, sensorPosition, 'hole')
//    causeProbabilities_posterior['infinity'] = measurementLikelihoodGivenInfinity * causeProbFromInfinity_prior
//
//    assert np.abs(np.sum(causeProbabilities_prior) + causeProbFromInfinity_prior - 1) < 1e-10
//    eta = np.sum(causeProbabilities_posterior['onRay']) + causeProbabilities_posterior['infinity']
//    causeProbabilities_posterior['onRay'] = causeProbabilities_posterior['onRay'] / eta
//    causeProbabilities_posterior['infinity'] = causeProbabilities_posterior['infinity'] / eta
//    assert np.abs(
//            np.sum(causeProbabilities_posterior['onRay']) + causeProbabilities_posterior['infinity'] - 1) < 1e-10
//    return (causeProbabilities_posterior, cause_global_ids)

    return icm;
}