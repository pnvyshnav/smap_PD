#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "../include/StereoCameraSensor.h"


StereoCameraSensor::StereoCameraSensor(Parameters::Vec3Type position, Parameters::Vec3Type orientation)
        : FakeSensor(position, orientation)
{
#if defined(FAKE_3D)
    auto direction = Eigen::Vector3f(orientation.x(), orientation.y(), orientation.z());

    double vFactor = Parameters::StereoCameraVerticalFOV / Parameters::StereoCameraVerticalPixels;
    double hFactor = Parameters::StereoCameraHorizontalFOV / Parameters::StereoCameraHorizontalPixels;
    for (unsigned int vp = 0; vp < Parameters::StereoCameraVerticalPixels; ++vp)
    {
        double angleV = -Parameters::StereoCameraVerticalFOV/2. + vp * vFactor;
        for (unsigned int hp = 0; hp < Parameters::StereoCameraHorizontalPixels; ++hp)
        {
            double angleH = -Parameters::StereoCameraHorizontalFOV/2. + hp * hFactor;
            auto rotHorizontal = Eigen::AngleAxis<float>((float) angleH, Eigen::Vector3f(0, 0, 1));
            auto rotVertical = Eigen::AngleAxis<float>((float) angleV, Eigen::Vector3f(1, 0, 0));
            Eigen::Vector3f rotated = rotVertical * rotHorizontal * (direction);
            _pixelSensors.push_back(PixelSensor(position, Parameters::Vec3Type(
                    rotated.x(), rotated.y(), rotated.z()
            )));
        }
    }
#elif defined(PLANNER_2D_TEST)
    auto direction = Eigen::Vector3f(orientation.x(), orientation.y(), orientation.z());

    double hFactor = Parameters::StereoCameraHorizontalFOV / Parameters::StereoCameraHorizontalPixels;
    for (unsigned int hp = 0; hp < Parameters::StereoCameraHorizontalPixels; ++hp)
    {
        double angleH = -Parameters::StereoCameraHorizontalFOV/2. + hp * hFactor;
        auto rotHorizontal = Eigen::AngleAxis<float>((float) angleH, Eigen::Vector3f(0, 0, 1));
        Eigen::Vector3f rotated = rotHorizontal * (direction);
        _pixelSensors.push_back(PixelSensor(position, Parameters::Vec3Type(
                rotated.x(), rotated.y(), rotated.z()
        )));
    }
#else
    _pixelSensors.push_back(PixelSensor(position, orientation));
#endif
}

Observation StereoCameraSensor::observe(TrueMap &trueMap) const
{
    std::vector<Measurement> measurements;
    for (auto &sensor : _pixelSensors)
        measurements.push_back(sensor.observe(trueMap).measurements().front());
    return Observation(measurements);
}

std::vector<PixelSensor> StereoCameraSensor::pixels() const
{
    return _pixelSensors;
}

Parameters::NumType StereoCameraSensor::likelihoodGivenCause(Measurement measurement, QVoxel causeVoxel) const
{
    // TODO improve architecture
    PixelSensor pixelSensor(measurement.sensor->position(), measurement.sensor->orientation());
    return pixelSensor.likelihoodGivenCause(measurement, causeVoxel);
}

void StereoCameraSensor::setPosition(const Parameters::Vec3Type &position)
{
    auto diff = this->position() - position;
    Sensor::setPosition(position);
    for (auto &pixel : _pixelSensors)
    {
        pixel._position += diff;
    }
}

void StereoCameraSensor::setOrientation(const Parameters::Vec3Type &orientation)
{
    auto oldOrientation = Eigen::Vector3f(Sensor::orientation().x(),
                                          Sensor::orientation().y(),
                                          Sensor::orientation().z());
    auto newOrientation = Eigen::Vector3f(orientation.x(), orientation.y(), orientation.z());
    auto rotation = Eigen::Quaternionf().setFromTwoVectors(oldOrientation, newOrientation);
    auto transform = Eigen::AngleAxis<float>(rotation);

    Sensor::setOrientation(orientation);
    for (auto &pixel : _pixelSensors)
    {
        Eigen::Vector3f po = Eigen::Vector3f(pixel._orientation.x(), pixel._orientation.y(), pixel._orientation.z());
        po = transform * po;
        pixel._orientation = Parameters::Vec3Type(po.x(), po.y(), po.z());
    }
}

