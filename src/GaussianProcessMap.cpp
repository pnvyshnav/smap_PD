#include "../include/GaussianProcessMap.h"

#include <ecl/time.hpp>

ecl::StopWatch stopWatchGP;

GaussianProcessMap::GaussianProcessMap(std::string kernel) :
        _gp(DIMENSIONS, kernel)
{
    std::cout << "The " << kernel << " kernel expects " << _gp.covf().get_param_dim() << " parameters." << std::endl;
    Eigen::VectorXd params(_gp.covf().get_param_dim());
    params << -1., -1., -0.5; //1.0, 0.5; // characteristic length scale, signal variance
    _gp.covf().set_loghyper(params);
}

bool GaussianProcessMap::update(const Observation &observation)
{
    stopWatchGP.restart();
    for (auto measurement : observation.measurements())
    {
        // Every voxel size length, sample from measurement ray.
        // The ray's end point is the occupied sample.
        for (auto &rayPoint : measurement.sensor.discretized(measurement.value))
        {
            double point[DIMENSIONS] = {};
            point[0] = rayPoint.position.x();
            point[1] = rayPoint.position.y();
#if DIMENSIONS > 2
            point[2] = rayPoint.position.z();
#endif
            _gp.add_pattern(point, rayPoint.occupied ? 1 : -2);
        }
    }
    std::cout << "Time to update GP map: " << stopWatchGP.elapsed() << std::endl;
    return true;
}

Belief GaussianProcessMap::belief(const octomap::point3d &position)
{
    double point[DIMENSIONS] = {};
    point[0] = position.x();
    point[1] = position.y();
#if DIMENSIONS > 2
    point[2] = position.z();
#endif
    return Belief((_gp.f(point) + 1.) / 2., _gp.var(point));
//    return Belief(_gp.f(point), _gp.var(point));
}
