#include "../include/GaussianProcessMap.h"

#include <ecl/time.hpp>

ecl::StopWatch stopWatchGP;

GaussianProcessMap::GaussianProcessMap(std::string kernel) :
        _gp(DIMENSIONS, kernel)
{
    std::cout << "The " << kernel << " kernel expects " << _gp.covf().get_param_dim() << " parameters." << std::endl;
    Eigen::VectorXd params(_gp.covf().get_param_dim());
    params << 0.0, 0.0, -1.0; //1.0, 0.5; // characteristic length scale, signal variance
    _gp.covf().set_loghyper(params);
}

bool GaussianProcessMap::update(const Observation &observation)
{
    // TODO this idea was taken from Hilbert Maps ...
    double point[DIMENSIONS] = {};
    stopWatchGP.restart();
    for (auto measurement : observation.measurements())
    {
        // Every voxel size length, sample from measurement ray.
        // The ray's end point is the occupied sample.
        for (double x = 0; x <= measurement.value - 3. * Parameters::voxelSize; x += 3. * Parameters::voxelSize)
        {
            point[0] = measurement.sensor.position.x() + x * measurement.sensor.orientation.x();
            point[1] = measurement.sensor.position.y() + x * measurement.sensor.orientation.y();
#if DIMENSIONS > 2
            point[2] = measurement.sensor.position.z() + x * measurement.sensor.orientation.z();
#endif
            if (point[0] < Parameters::xMin || point[0] > Parameters::xMax ||
                point[1] < Parameters::xMin || point[1] > Parameters::xMax ||
                point[2] < Parameters::xMin || point[2] > Parameters::xMax)
            {
                break;
            }
            _gp.add_pattern(point, 0); // free
        }
//        std::cout << measurement.sensor.position.x() << " + " << measurement.value << " * " << measurement.sensor.orientation.x() << " = " << (measurement.sensor.position.x() + measurement.value * measurement.sensor.orientation.x()) << std::endl;
        point[0] = measurement.sensor.position.x() + measurement.value * measurement.sensor.orientation.x();
        point[1] = measurement.sensor.position.y() + measurement.value * measurement.sensor.orientation.y();
#if DIMENSIONS > 2
        point[2] = measurement.sensor.position.z() + measurement.value * measurement.sensor.orientation.z();
#endif
        if (point[0] < Parameters::xMin || point[0] > Parameters::xMax ||
            point[1] < Parameters::xMin || point[1] > Parameters::xMax ||
            point[2] < Parameters::xMin || point[2] > Parameters::xMax)
        {
            continue;
        }
        _gp.add_pattern(point, 1); // occupied
    }
//    std::cout << "last stored value at "
//              << point[0] << "," << point[1] << ": "
//                                             << _gp.f(point) << std::endl;
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
    return Belief(_gp.f(point), _gp.var(point));
}
