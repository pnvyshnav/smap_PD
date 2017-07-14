#include "../include/Visualizer.h"
#include "../include/TrueMap.h"
#include "../include/BeliefMap.h"
#include "../include/Sensor.h"
#include "../include/LogOddsMap.h"
#include "../include/Parameters.hpp"
#include "../include/StereoCameraSensor.h"
#include "../include/FakeRobot.hpp"
#include "../include/GaussianProcessMap.h"

#include <visualization_msgs/Marker.h>
#include <nav_msgs/GridCells.h>
#include <ecl/time.hpp>
#include <cmath>

ecl::StopWatch stopWatchVisualizer;

void Visualizer::render()
{
    ROS_INFO("Render");
    ros::Rate loop_rate(0.1);

    while (ros::ok())
    {
//        publishTrueMap(_lastTrueMap);
//        publishTrueMap2dSlice(_lastTrueMap);
//        publishBeliefMapFull(_lastBeliefMap);
        loop_rate.sleep();
    }

    ros::spin();
}

Visualizer::Visualizer() : _counter(0)
{
    srand(0);

    nodeHandle = new ros::NodeHandle;
    trueMapPublisher = nodeHandle->advertise<visualization_msgs::MarkerArray>("true_map", 1);
    trueMap2dPublisher = nodeHandle->advertise<nav_msgs::GridCells>("true_map_2d", 10);
    logOddsMapPublisher = nodeHandle->advertise<visualization_msgs::MarkerArray>("logodds_map", 10);
    beliefMapPublisher = nodeHandle->advertise<visualization_msgs::MarkerArray>("belief_map", 10);
    gaussianProcessMapPublisher = nodeHandle->advertise<visualization_msgs::MarkerArray>("gp_map", 10);
    sensorPublisher = nodeHandle->advertise<visualization_msgs::Marker>("sensor", 10);
    stereoCameraSensorPublisher = nodeHandle->advertise<visualization_msgs::MarkerArray>("stereo_cam", 10);
    rayVoxelPublisher = nodeHandle->advertise<visualization_msgs::MarkerArray>("ray_voxels", 10);
    trajectoryVoxelsPublisher = nodeHandle->advertise<visualization_msgs::MarkerArray>("trajectoryVoxels", 10);
    finalTrajectoryPublisher = nodeHandle->advertise<visualization_msgs::Marker>("hist_positions", 10);
    trajectoryPublisher = nodeHandle->advertise<visualization_msgs::MarkerArray>("trajectory", 10);
    observationPublisher = nodeHandle->advertise<visualization_msgs::MarkerArray>("observation", 10);
}

Visualizer::~Visualizer()
{
    delete nodeHandle;
}

void Visualizer::publishTrueMap(const Observable *visualizable)
{
    if (!visualizable)
        return;

    //ROS_INFO("Publishing True Map");

    auto trueMap = (TrueMap *) visualizable;
    ros::Rate loop_rate(PaintRate);
    loop_rate.sleep();
//    ROS_INFO("Visualizing true map");

    visualization_msgs::MarkerArray cells;
    for (unsigned int x = 0; x < Parameters::voxelsPerDimensionX; ++x) {
        for (unsigned int y = 0; y < Parameters::voxelsPerDimensionY; ++y) {
            for (unsigned int z = 0; z < Parameters::voxelsPerDimensionZ; ++z) {
                auto _x = Parameters::xMin + x * Parameters::voxelSize;
                auto _y = Parameters::yMin + y * Parameters::voxelSize;
                auto _z = Parameters::zMin + (z + 0.5) * Parameters::voxelSize;
                QTrueVoxel voxel = trueMap->query(_x, _y, _z);
                if (!voxel.node())
                    continue;

                visualization_msgs::Marker cell;
#ifdef REAL_3D
                if (trueMap->getVoxelMean(voxel) < 0.3)
                    continue;
#else
                if (z > 12 && (trueMap->getVoxelMean(voxel) < 0.5 && voxel.position.norm() < 1.5 || voxel.position.norm() < 0.6))
                    // remove voxel
                    continue;
                else
                    cell.action = 0;
#endif

                cell.id = (int) voxel.hash;
                cell.type = visualization_msgs::Marker::CUBE;
                cell.header.frame_id = "map";
                cell.scale.x = Parameters::voxelSize;
                cell.scale.y = Parameters::voxelSize;
                cell.scale.z = Parameters::voxelSize;
                cell.color.a = 1.0f;
                cell.color.r = (float)(1. - trueMap->getVoxelMean(voxel) * .5);
                cell.color.g = (float)(1. - trueMap->getVoxelMean(voxel) * .5); //(int)std::round(voxel.node()->getOccupancy());
                cell.color.b = (float)(1. - trueMap->getVoxelMean(voxel) * .5);
                cell.pose.position.x = _x;
                cell.pose.position.y = _y;
                cell.pose.position.z = _z;
                cells.markers.push_back(cell);
            }
        }
    }

    ROS_INFO("Publishing %d cells", (int)cells.markers.size());
    trueMapPublisher.publish(cells);
    ros::spinOnce();
}

void Visualizer::publishTrueMap2dSlice(const Observable *visualizable, unsigned int z)
{
    if (!visualizable)
        return;

    auto trueMap = (TrueMap*) visualizable;
    ros::Rate loop_rate(PaintRate);
    loop_rate.sleep();

    nav_msgs::GridCells grid;
    grid.header.frame_id = "map";
    grid.cell_height = (float) Parameters::voxelSize;
    grid.cell_width = (float) Parameters::voxelSize;

    // TODO fix this
    auto _z = 0; ///*Parameters::zMin +*/ (z + Parameters::voxelsPerDimensionZ/2) * Parameters::voxelSize;
    for (unsigned int x = 0; x < Parameters::voxelsPerDimensionX; ++x)
    {
        for (unsigned int y = 0; y < Parameters::voxelsPerDimensionY; ++y)
        {
            auto _x = Parameters::xMin + x * Parameters::voxelSize;
            auto _y = Parameters::yMin + y * Parameters::voxelSize;
            QTrueVoxel voxel = trueMap->query(_x, _y, _z);

            if (!voxel.node() || trueMap->getVoxelMean(voxel) < 0.5)
            {
//                std::cout << "Voxel " << _x << " " << _y << " " << _z << " valid? " << !!voxel.node()
//                                             << "   occupancy: "
//                                             << trueMap->getVoxelMean(voxel)
//                                             << std::endl;
                continue;
            }

            geometry_msgs::Point p;
            p.x = _x;
            p.y = _y;
            p.z = _z;

            grid.cells.push_back(p);
        }
    }

    trueMap2dPublisher.publish(grid);
    ros::spinOnce();
}

void Visualizer::publishLogOddsMap(const Observable *visualizable)
{
    if (!visualizable)
        return;

    auto logOddsMap = (LogOddsMap*) visualizable;
    //ros::Rate loop_rate(PaintRate);
    //loop_rate.sleep();

    visualization_msgs::MarkerArray cells;
    for (auto &voxel : logOddsMap->updatedVoxels())
    {
        visualization_msgs::Marker cell;
        // TODO do not remove voxels by z position
#ifndef FAKE_2D
        if (voxel.node()->getOccupancy() < 0.52
#ifndef FAKE_3D
            || voxel.position.z() < 0.4 || voxel.position.z() > 1.5
#endif
                )
        {
            // remove voxel
            cell.action = 2;
            cell.id = (int) voxel.hash;
        }
        else
#endif
        {
            cell.action = 0;
            cell.id = (int) voxel.hash;
            cell.type = visualization_msgs::Marker::CUBE;

            cell.header.frame_id = "map";
            cell.scale.x = Parameters::voxelSize;
            cell.scale.y = Parameters::voxelSize;
            cell.scale.z = Parameters::voxelSize;
            cell.color.a = 1;
            float intensity = (float) (1.0 - voxel.node()->getOccupancy());
            cell.color.r = intensity;
            cell.color.g = intensity;
            cell.color.b = intensity;
            cell.pose.position.x = voxel.position.x();
            cell.pose.position.y = voxel.position.y();
            cell.pose.position.z = voxel.position.z();
        }

        cells.markers.push_back(cell);
    }

    logOddsMapPublisher.publish(cells);
}

void Visualizer::publishBeliefMap(const Observable *visualizable)
{
    if (!visualizable)
        return;

    auto beliefMap = (BeliefMap*) visualizable;
    //ros::Rate loop_rate(PaintRate);
    //loop_rate.sleep();

    visualization_msgs::MarkerArray cells;
    for (auto &voxel : beliefMap->updatedVoxels())
    {
        visualization_msgs::Marker cell;
        // TODO do not remove voxels by z position
//#ifndef FAKE_2D
//        if (voxel.node()->getValue()->mean() < 0.52
//#ifndef FAKE_3D
////            || voxel.position.z() < 0.4 || voxel.position.z() > 1.5
//#endif
//            )
//        {
//            // remove voxel
//            cell.action = 2;
//            cell.id = (int) voxel.hash;
//        }
//        else
//#endif
        {
            cell.action = 0;
            cell.id = (int) voxel.hash;
            cell.type = visualization_msgs::Marker::CUBE;
            cell.header.frame_id = "map";
            cell.scale.x = Parameters::voxelSize;
            cell.scale.y = Parameters::voxelSize;
            cell.scale.z = Parameters::voxelSize;
            cell.color.a = 1;
            float intensity = (float) (1.0 - voxel.node()->getValue().mean());
            cell.color.r = intensity;
            cell.color.g = intensity;
            cell.color.b = intensity;
            cell.pose.position.x = voxel.position.x();
            cell.pose.position.y = voxel.position.y();
            cell.pose.position.z = voxel.position.z();
        }

        cells.markers.push_back(cell);
    }

    beliefMapPublisher.publish(cells);

    // Publish ICM's ray voxels
//    visualization_msgs::MarkerArray rayVoxels;
//    visualization_msgs::Marker clearVoxel;
//    clearVoxel.action = 3; // clear all
//    clearVoxel.header.frame_id = "map";
//    rayVoxels.markers.push_back(clearVoxel);
//    if (beliefMap->icm != NULL)
//    {
//        for (auto &key : beliefMap->icm->ray)
//        {
//            QBeliefVoxel beliefVoxel = beliefMap->query(key);
//
//            visualization_msgs::Marker rayVoxel;
//            rayVoxel.action = 0;
//            rayVoxel.id = (int) beliefVoxel.hash + 1000; // do not overwrite BeliefMap voxels
//            rayVoxel.type = visualization_msgs::Marker::CUBE;
//            rayVoxel.header.frame_id = "map";
//            rayVoxel.scale.x = Parameters::voxelSize;
//            rayVoxel.scale.y = Parameters::voxelSize;
//            rayVoxel.scale.z = Parameters::voxelSize;
//            rayVoxel.color.a = 0.7;
//
//            if (beliefVoxel.type != GEOMETRY_VOXEL)
//            {
//                rayVoxel.color.r = 0.2;
//                rayVoxel.color.g = 0.2;
//                rayVoxel.color.b = 0.2;
//            }
//            else
//            {
//                rayVoxel.color.r = 0.9;
//                rayVoxel.color.g = 0.4;
//                rayVoxel.color.b = 0.0;
//            }
//
//            rayVoxel.pose.position.x = beliefVoxel.position.x();
//            rayVoxel.pose.position.y = beliefVoxel.position.y();
//            rayVoxel.pose.position.z = beliefVoxel.position.z();
//            rayVoxels.markers.push_back(rayVoxel);
//        }
//    }
//
//    rayVoxelPublisher.publish(rayVoxels);

    //ros::spinOnce();
}

void Visualizer::publishBeliefMapFull(const Observable *visualizable, bool visualizeStd)
{
    if (!visualizable)
        return;

    auto beliefMap = (BeliefMap*) visualizable;
    ros::Rate loop_rate(PaintRate);
    loop_rate.sleep();

    visualization_msgs::MarkerArray cells;

//    int id = 23423;
//    for (float y = -1; y <= 1.01; y += Parameters::voxelSize)
//    {
//        Parameters::Vec3Type pos(0, y, 0);
//
//        visualization_msgs::Marker cell;
//        cell.id = id++;
//        cell.action = 0;
//        cell.type = visualization_msgs::Marker::CUBE;
//        cell.header.frame_id = "map";
//        cell.scale.x = Parameters::voxelSize;
//        cell.scale.y = Parameters::voxelSize;
//        cell.scale.z = Parameters::voxelSize;
//        cell.color.a = 1.0;
//
//        double r, g, b;
//        double hue = (y+1.)/4. * 1.3;
//        std::cout << y << std::endl;
//        hsl2rgb(hue, 1, .6, r, g, b);
//        cell.color.r = (float) r;
//        cell.color.g = (float) g;
//        cell.color.b = (float) b;
//        cell.pose.position.x = pos.x();
//        cell.pose.position.y = pos.y();
//        cell.pose.position.z = pos.z();
//        cells.markers.push_back(cell);
//    }

    for (unsigned int x = 0; x < Parameters::voxelsPerDimensionX; ++x)
    {
        for (unsigned int y = 0; y < Parameters::voxelsPerDimensionY; ++y)
        {
            for (unsigned int z = 0; z < Parameters::voxelsPerDimensionZ; ++z)
            {
                auto _x = Parameters::xMin + x * Parameters::voxelSize;
                auto _y = Parameters::yMin + y * Parameters::voxelSize;
                auto _z = Parameters::zMin + z * Parameters::voxelSize;
                Parameters::Vec3Type pos(_x, _y, _z);
                QBeliefVoxel voxel = beliefMap->query(_x, _y, _z);

                visualization_msgs::Marker cell;
                cell.id = (int) voxel.hash;
//                if (beliefMap->getVoxelMean(voxel) < 0.49 && pos.norm() < 1.5 || pos.norm() < 0.6)
//                    // remove voxel
//                    cell.action = 2;
//                else
                    cell.action = 0;
                cell.type = visualization_msgs::Marker::CUBE;
                cell.header.frame_id = "map";
                cell.scale.x = Parameters::voxelSize;
                cell.scale.y = Parameters::voxelSize;
                cell.scale.z = Parameters::voxelSize;
                cell.color.a = 1.0;

                double r, g, b;
                if (voxel.type == GEOMETRY_VOXEL)
                {
                    if (visualizeStd)
                    {
                        //TODO scale std dev, since belief voxel have initial variance of 0.085
                        double hue = 0.5/std::sqrt(0.085) * std::sqrt(voxel.node()->getValue().variance()) * 1.3;
                        hsl2rgb(hue, 1, .6, r, g, b);
                    }
                    else
                        r = g = b = (float) (1.0 - voxel.node()->getValue().mean());
                }
                else
                    continue;
                cell.color.r = (float) r;
                cell.color.g = (float) g;
                cell.color.b = (float) b;
                cell.pose.position.x = voxel.position.x();
                cell.pose.position.y = voxel.position.y();
                cell.pose.position.z = voxel.position.z();
                cells.markers.push_back(cell);
            }
        }
    }

    beliefMapPublisher.publish(cells);

    // Clear ICM's ray voxels
    visualization_msgs::MarkerArray rayVoxels;
    visualization_msgs::Marker clearVoxel;
    clearVoxel.action = 3; // clear all
    clearVoxel.header.frame_id = "map";
    rayVoxels.markers.push_back(clearVoxel);
    rayVoxelPublisher.publish(rayVoxels);

    ros::spinOnce();
}

void Visualizer::publishBeliefInconsistencyMapFull(const Observable *visualizable, TrueMap &trueMap, double k)
{
    if (!visualizable)
        return;

    auto beliefMap = (BeliefMap*) visualizable;
    ros::Rate loop_rate(PaintRate);
    loop_rate.sleep();

    visualization_msgs::MarkerArray cells;

    for (unsigned int x = 0; x < Parameters::voxelsPerDimensionX; ++x)
    {
        for (unsigned int y = 0; y < Parameters::voxelsPerDimensionY; ++y)
        {
            for (unsigned int z = 0; z < Parameters::voxelsPerDimensionZ; ++z)
            {
                auto _x = Parameters::xMin + x * Parameters::voxelSize;
                auto _y = Parameters::yMin + y * Parameters::voxelSize;
                auto _z = Parameters::zMin + z * Parameters::voxelSize;
                Parameters::Vec3Type pos(_x, _y, _z);
                QBeliefVoxel voxel = beliefMap->query(_x, _y, _z);

                visualization_msgs::Marker cell;
                cell.id = (int) voxel.hash;
                cell.action = 0;
                cell.type = visualization_msgs::Marker::CUBE;
                cell.header.frame_id = "map";
                cell.scale.x = Parameters::voxelSize;
                cell.scale.y = Parameters::voxelSize;
                cell.scale.z = Parameters::voxelSize;
                cell.color.a = 1.0;

                double r, g, b = 0;
                if (voxel.type == GEOMETRY_VOXEL)
                {
                    QTrueVoxel truth = trueMap.query(_x, _y, _z);
                    double error = std::abs(beliefMap->getVoxelMean(voxel)
                                            - trueMap.getVoxelMean(truth));
//                    std::cout << error << std::endl;
                    double std = beliefMap->getVoxelStd(voxel);
                    double inconsistency = error - k * std;
                    if (inconsistency > 0)
                    {
                        r = std::min(1., inconsistency);
                        g = std::min(1., inconsistency);
                        b = std::min(1., inconsistency);
                    }
                    else
                        r = g = b = 0;
                }
                else
                    continue;
                cell.color.r = (float) r;
                cell.color.g = (float) g;
                cell.color.b = (float) b;
                cell.pose.position.x = voxel.position.x();
                cell.pose.position.y = voxel.position.y();
                cell.pose.position.z = voxel.position.z();
                cells.markers.push_back(cell);
            }
        }
    }

    beliefMapPublisher.publish(cells);

    // Clear ICM's ray voxels
    visualization_msgs::MarkerArray rayVoxels;
    visualization_msgs::Marker clearVoxel;
    clearVoxel.action = 3; // clear all
    clearVoxel.header.frame_id = "map";
    rayVoxels.markers.push_back(clearVoxel);
    rayVoxelPublisher.publish(rayVoxels);

    ros::spinOnce();
}

void Visualizer::publishLogOddsMapFull(const Observable *visualizable, bool visualizeStd)
{
    if (!visualizable)
        return;

    auto logOddsMap = (LogOddsMap*) visualizable;
    ros::Rate loop_rate(PaintRate);
    loop_rate.sleep();

    visualization_msgs::MarkerArray cells;
    octomap::OcTreeKey::KeyHash hasher;
    for (unsigned int x = 0; x < Parameters::voxelsPerDimensionX; ++x)
    {
        for (unsigned int y = 0; y < Parameters::voxelsPerDimensionY; ++y)
        {
            for (unsigned int z = 0; z < Parameters::voxelsPerDimensionZ; ++z)
            {
                auto _x = Parameters::xMin + x * Parameters::voxelSize;
                auto _y = Parameters::yMin + y * Parameters::voxelSize;
                auto _z = Parameters::zMin + z * Parameters::voxelSize;
                Parameters::Vec3Type pos(_x, _y, _z);
                QTrueVoxel voxel = logOddsMap->query(_x, _y, _z);

                visualization_msgs::Marker cell;
                cell.id = 1379 + (int) hasher(octomap::OcTreeKey(x, y, z));
                cell.action = 0;

                double r, g, b;
                if (voxel.type == GEOMETRY_VOXEL)
                {
                    if (visualizeStd)
                    {
                        double hue = logOddsMap->getVoxelStd(voxel) * 1.3;
                        hsl2rgb(hue, 1, .6, r, g, b);
                    }
                    else
                        r = g = b = (float) (1.0 - logOddsMap->getVoxelMean(voxel));
                }
                else
                {
                    r = g = b = (float) (1.0 - Parameters::priorMean);
//                    continue;
                }
                cell.color.r = (float) r;
                cell.color.g = (float) g;
                cell.color.b = (float) b;
                cell.type = visualization_msgs::Marker::CUBE;
                cell.header.frame_id = "map";
                cell.scale.x = Parameters::voxelSize;
                cell.scale.y = Parameters::voxelSize;
                cell.scale.z = Parameters::voxelSize;
                cell.color.a = 1.0;
                cell.pose.position.x = pos.x();
                cell.pose.position.y = pos.y();
                cell.pose.position.z = pos.z();
                cells.markers.push_back(cell);
            }
        }
    }

    logOddsMapPublisher.publish(cells);

    // Clear ICM's ray voxels
    visualization_msgs::MarkerArray rayVoxels;
    visualization_msgs::Marker clearVoxel;
    clearVoxel.action = 3; // clear all
    clearVoxel.header.frame_id = "map";
    rayVoxels.markers.push_back(clearVoxel);
    rayVoxelPublisher.publish(rayVoxels);

    ros::spinOnce();
}

void Visualizer::publishLogOddsInconsistencyMapFull(const Observable *visualizable, TrueMap &trueMap, double k)
{
    if (!visualizable)
        return;

    auto logOddsMap = (LogOddsMap*) visualizable;
    ros::Rate loop_rate(PaintRate);
    loop_rate.sleep();

    visualization_msgs::MarkerArray cells;
    octomap::OcTreeKey::KeyHash hasher;
    for (unsigned int x = 0; x < Parameters::voxelsPerDimensionX; ++x)
    {
        for (unsigned int y = 0; y < Parameters::voxelsPerDimensionY; ++y)
        {
            for (unsigned int z = 0; z < Parameters::voxelsPerDimensionZ; ++z)
            {
                auto _x = Parameters::xMin + x * Parameters::voxelSize;
                auto _y = Parameters::yMin + y * Parameters::voxelSize;
                auto _z = Parameters::zMin + z * Parameters::voxelSize;
                Parameters::Vec3Type pos(_x, _y, _z);
                QTrueVoxel voxel = logOddsMap->query(_x, _y, _z);

                visualization_msgs::Marker cell;
                cell.id = 1379 + (int) hasher(octomap::OcTreeKey(x, y, z));
                cell.action = 0;

                double r, g, b;
                if (voxel.type == GEOMETRY_VOXEL)
                {
                    QTrueVoxel truth = trueMap.query(_x, _y, _z);
                    double error = std::abs(logOddsMap->getVoxelMean(voxel)
                                            - trueMap.getVoxelMean(truth));
//                    std::cout << error << std::endl;
                    double std = logOddsMap->getVoxelStd(voxel);
                    double inconsistency = error - k * std;
                    if (inconsistency > 0)
                    {
                        r = std::min(1., inconsistency);
                        g = std::min(1., inconsistency);
                        b = std::min(1., inconsistency);
                    }
                    else
                        r = g = b = 0;
                }
                else
                    continue;
                cell.color.r = (float) r;
                cell.color.g = (float) g;
                cell.color.b = (float) b;
                cell.type = visualization_msgs::Marker::CUBE;
                cell.header.frame_id = "map";
                cell.scale.x = Parameters::voxelSize;
                cell.scale.y = Parameters::voxelSize;
                cell.scale.z = Parameters::voxelSize;
                cell.color.a = 1.0;
                cell.pose.position.x = pos.x();
                cell.pose.position.y = pos.y();
                cell.pose.position.z = pos.z();
                cells.markers.push_back(cell);
            }
        }
    }

    logOddsMapPublisher.publish(cells);

    // Clear ICM's ray voxels
    visualization_msgs::MarkerArray rayVoxels;
    visualization_msgs::Marker clearVoxel;
    clearVoxel.action = 3; // clear all
    clearVoxel.header.frame_id = "map";
    rayVoxels.markers.push_back(clearVoxel);
    rayVoxelPublisher.publish(rayVoxels);

    ros::spinOnce();
}

void Visualizer::publishSensor(const Observable *visualizable)
{
    auto sensor = (Sensor*) visualizable;
    ros::Rate loop_rate(PaintRate);

    loop_rate.sleep();

    visualization_msgs::Marker arrow;
    arrow.action = 0;
    arrow.id = (int) visualizable->observableId();
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.header.frame_id = "map";
    arrow.scale.x = 0.05;
    arrow.scale.y = 0.05;
    arrow.scale.z = 0.05;
    arrow.color.a = 1;
    arrow.color.r = 1;
    arrow.color.g = 0;
    arrow.color.b = 0;
    geometry_msgs::Point base;
    base.x = sensor->position().x();
    base.y = sensor->position().y();
    base.z = sensor->position().z();
    geometry_msgs::Point target;
    target.x = sensor->position().x() + Parameters::sensorRange * sensor->orientation().x();
    target.y = sensor->position().y() + Parameters::sensorRange * sensor->orientation().y();
    target.z = sensor->position().z() + Parameters::sensorRange * sensor->orientation().z();
#ifdef FAKE_2D
    base.z += 0.1;
    target.z += 0.1;
#endif
    arrow.points.push_back(base);
    arrow.points.push_back(target);

    sensorPublisher.publish(arrow);

    ros::spinOnce();
}

void Visualizer::publishStereoCameraSensor(const Observable *visualizable)
{
    auto camera = (StereoCameraSensor*) visualizable;

    visualization_msgs::MarkerArray arrows;
    unsigned int i = 0;
    for (auto &sensor : camera->pixels())
    {
        visualization_msgs::Marker arrow;
        arrow.action = 0;
        arrow.id = (int) visualizable->observableId() + 1000 + i;
        arrow.type = visualization_msgs::Marker::ARROW;
        arrow.header.frame_id = "map";
        arrow.scale.x = 0.02;
        arrow.scale.y = 0.02;
        arrow.scale.z = 0.02;
        arrow.color.a = 1;
        arrow.color.r = 1;
        arrow.color.g = 0;
        arrow.color.b = 0;
        geometry_msgs::Point base;
        base.x = sensor.position().x();
        base.y = sensor.position().y();
        base.z = sensor.position().z();
        geometry_msgs::Point target;
        target.x = sensor.position().x() + Parameters::sensorRange * sensor.orientation().x();
        target.y = sensor.position().y() + Parameters::sensorRange * sensor.orientation().y();
        target.z = sensor.position().z() + Parameters::sensorRange * sensor.orientation().z();
#ifdef FAKE_2D
        base.z += 0.1;
        target.z += 0.1;
#endif
        arrow.points.push_back(base);
        arrow.points.push_back(target);

        arrows.markers.push_back(arrow);
        ++i;
    }

    stereoCameraSensorPublisher.publish(arrows);
}

void Visualizer::publishRay(TrueMap &trueMap, Sensor &sensor)
{
    octomap::KeyRay ray;
    trueMap.computeRayKeys(sensor.position(), sensor.position() + sensor.orientation() * Parameters::sensorRange, ray);

    ros::Rate loop_rate(PaintRate);

    int step = 0;
    while (ros::ok())
    {
        loop_rate.sleep();

        // Publish ICM's ray voxels
        visualization_msgs::MarkerArray rayVoxels;
        visualization_msgs::Marker clearVoxel;
        clearVoxel.action = 3; // clear all
        clearVoxel.header.frame_id = "map";
        //rayVoxels.markers.push_back(clearVoxel);
        unsigned int i = 0;
        for (auto &key : ray)
        {
            QTrueVoxel trueVoxel = trueMap.query(key);
            if (trueVoxel.type != GEOMETRY_VOXEL)
                continue;

            visualization_msgs::Marker rayVoxel;
            rayVoxel.action = 0;
            rayVoxel.id = (int) trueVoxel.hash;
            rayVoxel.type = visualization_msgs::Marker::CUBE;
            rayVoxel.header.frame_id = "map";
            rayVoxel.scale.x = Parameters::voxelSize;
            rayVoxel.scale.y = Parameters::voxelSize;
            rayVoxel.scale.z = Parameters::voxelSize;
            rayVoxel.color.a = 1.0;
            rayVoxel.color.r = 0.4;
            rayVoxel.color.g = 0.9;
            rayVoxel.color.b = 0.0;
            rayVoxel.pose.position.x = trueVoxel.position.x();
            rayVoxel.pose.position.y = trueVoxel.position.y();
            rayVoxel.pose.position.z = trueVoxel.position.z();
            rayVoxels.markers.push_back(rayVoxel);
            i+=1;
            if (i >= ray.size())
                break;
        }

        rayVoxelPublisher.publish(rayVoxels);

        ros::spinOnce();
        ++step;
        if (step > 10)
            break;
    }
}

void Visualizer::publishFakeRobot(const Observable *visualizable, const TrueMap *trueMap)
{
    auto robot = (FakeRobot<>*) visualizable;
    ros::Rate loop_rate(PaintRate);

    loop_rate.sleep();
    visualization_msgs::MarkerArray markers;

    auto colors = std::vector<std::vector<double> > {
            {0, 0, 1}, // TODO remove
            {1, 0, 0},
            {0, 0.4, 0},
            {0, 0, 1}
    };

#if defined(PLANNER_2D_TEST)
    visualization_msgs::MarkerArray allTrajectoryVoxels;

    int counter = 0;
    unsigned int splineId = 0;
//    for (auto &spline : robot->splines())
//    {
    visualization_msgs::Marker marker;
    marker.action = 0;
    marker.id = (int) visualizable->observableId() + counter++;
    unsigned int color = (unsigned int) (splineId % colors.size());
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.header.frame_id = "map";
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.a = 1;
    marker.color.r = (float)colors[color][0];
    marker.color.g = (float)colors[color][1];
    marker.color.b = (float)colors[color][2];

    visualization_msgs::Marker wayPoints;
    wayPoints.action = 0;
    wayPoints.id = (int) visualizable->observableId() + counter++;
    wayPoints.type = visualization_msgs::Marker::POINTS;
    wayPoints.header.frame_id = "map";
    wayPoints.scale.x = 0.02;
    wayPoints.scale.y = 0.02;
    wayPoints.scale.z = 0.02;
    wayPoints.color.a = 1;
    wayPoints.color.r = (float)colors[color][0];
    wayPoints.color.g = (float)colors[color][1];
    wayPoints.color.b = (float)colors[color][2];

    auto trajectory = Trajectory(robot->trajectory());
#ifdef SIMULATION_TIME
    for (double x = 0; x <= Parameters::SimulationFinalTime; x += Parameters::SimulationTimeStep)
        {
            auto result = trajectory.evaluateAtTime(x);
#else
    for (double x = 0; x <= 1.01; x += 0.05)
    {
        auto result = trajectory.evaluate(x);
#endif
        geometry_msgs::Point p;
        p.x = result.point.x;
        p.y = result.point.y;
        p.z = 0.5;
        marker.points.push_back(p);
        p.z = 0.51;
        wayPoints.points.push_back(p);
    }
    markers.markers.push_back(marker);
    markers.markers.push_back(wayPoints);

    // visualize all trajectory voxels
    visualization_msgs::Marker trajectoryVoxels;
    trajectoryVoxels.action = 0;
    trajectoryVoxels.id = (int) 5;
    trajectoryVoxels.type = visualization_msgs::Marker::POINTS;
    trajectoryVoxels.header.frame_id = "map";
    trajectoryVoxels.scale.x = 0.1;
    trajectoryVoxels.scale.y = 0.1;
    trajectoryVoxels.scale.z = 0.1;
    trajectoryVoxels.color.a = 0.5;
    trajectoryVoxels.color.r = (float)colors[color][0];
    trajectoryVoxels.color.g = (float)colors[color][1];
    trajectoryVoxels.color.b = (float)colors[color][2];
    //robot->selectSpline(splineId); // TODO messes up simulation
    for (auto &coords : robot->trajectory().splineVoxelPositions(*trueMap))
    {
        geometry_msgs::Point p;
        p.x = coords.x();
        p.y = coords.y();
        p.z = 0.51;
        trajectoryVoxels.points.push_back(p);
    }
    allTrajectoryVoxels.markers.push_back(trajectoryVoxels);

    // visualize all trajectory voxels
    visualization_msgs::Marker trajectoryFutureVoxels;
    trajectoryFutureVoxels.action = 0;
    trajectoryFutureVoxels.id = (int) 10;
    trajectoryFutureVoxels.type = visualization_msgs::Marker::POINTS;
    trajectoryFutureVoxels.header.frame_id = "map";
    trajectoryFutureVoxels.scale.x = 0.1;
    trajectoryFutureVoxels.scale.y = 0.1;
    trajectoryFutureVoxels.scale.z = 0.1;
    trajectoryFutureVoxels.color.a = 0.6;
    trajectoryFutureVoxels.color.r = 0.8;
    trajectoryFutureVoxels.color.g = 0.6;
    trajectoryFutureVoxels.color.b = 0.0;
    //robot->selectSpline(splineId); // TODO messes up simulation
    for (auto &key : robot->currentSplineFutureVoxels())
    {
        auto coords = trueMap->keyToCoord(key);
        geometry_msgs::Point p;
        p.x = coords.x();
        p.y = coords.y();
        p.z = 0.52;
        trajectoryFutureVoxels.points.push_back(p);
    }
    allTrajectoryVoxels.markers.push_back(trajectoryFutureVoxels);

    ++splineId;
//    }

    splinePublisher.publish(markers);
    trajectoryVoxelsPublisher.publish(allTrajectoryVoxels);

    visualization_msgs::Marker robotPosition;
    robotPosition.action = 0;
    robotPosition.id = 3473 + _counter++;
    robotPosition.type = visualization_msgs::Marker::SPHERE;
    robotPosition.header.frame_id = "map";
    robotPosition.pose.position.x = robot->position().x();
    robotPosition.pose.position.y = robot->position().y();
    robotPosition.pose.position.z = 0.6;
    robotPosition.scale.x = 0.02;
    robotPosition.scale.y = 0.02;
    robotPosition.scale.z = 0.02;
    robotPosition.color.a = 1.0;
    robotPosition.color.r = 0.9;
    robotPosition.color.g = 0.7;
    robotPosition.color.b = 0.0;
    finalTrajectoryPublisher.publish(robotPosition);
#endif

    ros::spinOnce();
}

void Visualizer::publishGaussianProcessMapFull(const Observable *visualizable, bool visualizeStd)
{
    if (!visualizable)
        return;

    auto gaussianProcessMap = (GaussianProcessMap*) visualizable;
    ros::Rate loop_rate(PaintRate);
    loop_rate.sleep();

    stopWatchVisualizer.restart();

    visualization_msgs::MarkerArray cells;
    for (unsigned int x = 0; x < Parameters::voxelsPerDimensionX; ++x)
    {
        for (unsigned int y = 0; y < Parameters::voxelsPerDimensionY; ++y)
        {
            for (unsigned int z = 0; z < Parameters::voxelsPerDimensionZ; ++z)
            {
                auto _x = Parameters::xMin + x * Parameters::voxelSize;
                auto _y = Parameters::yMin + y * Parameters::voxelSize;
                auto _z = Parameters::zMin + z * Parameters::voxelSize;
                Parameters::Vec3Type pos(_x, _y, _z);
                Belief belief = gaussianProcessMap->belief(pos);

                visualization_msgs::Marker cell;
                cell.id = (int) QVoxel::computeHash(TrueMap::coordToKey(pos));
                double mean = belief.mean();
//                std::cout << mean << std::endl;
//                mean = std::min(1., std::max(0., mean));
//                if (mean < 0.49 && pos.norm() < 1.5 || pos.norm() < 0.6)
//                    // remove belief
//                    cell.action = 2;
//                else
                    cell.action = 0;
                cell.type = visualization_msgs::Marker::CUBE;
                cell.header.frame_id = "map";
                cell.scale.x = Parameters::voxelSize;
                cell.scale.y = Parameters::voxelSize;
                cell.scale.z = Parameters::voxelSize;
                cell.color.a = 1.0;
                double r, g, b;
                if (visualizeStd)
                {
                    double hue = std::sqrt(belief.variance()) * GaussianProcessMap::StdDevScalingFactor * 1.3;
                    hsl2rgb(hue, 1, .6, r, g, b);
                }
                else
                    r = g = b = (float) (1.0 - belief.mean());

                cell.color.r = (float) r;
                cell.color.g = (float) g;
                cell.color.b = (float) b;
                cell.pose.position.x = _x;
                cell.pose.position.y = _y;
                cell.pose.position.z = _z;
                cells.markers.push_back(cell);
            }
        }
    }

    std::cout << "Time to query GP map: " << stopWatchVisualizer.elapsed() << std::endl;

    gaussianProcessMapPublisher.publish(cells);

    ros::spinOnce();
}

void Visualizer::publishGaussianProcessInconsistencyMapFull(const Observable *visualizable, TrueMap &trueMap, double k)
{
    if (!visualizable)
        return;

    auto gaussianProcessMap = (GaussianProcessMap *) visualizable;
    ros::Rate loop_rate(PaintRate);
    loop_rate.sleep();

    stopWatchVisualizer.restart();

    visualization_msgs::MarkerArray cells;
    for (unsigned int x = 0; x < Parameters::voxelsPerDimensionX; ++x)
    {
        for (unsigned int y = 0; y < Parameters::voxelsPerDimensionY; ++y)
        {
            for (unsigned int z = 0; z < Parameters::voxelsPerDimensionZ; ++z)
            {
                auto _x = Parameters::xMin + x * Parameters::voxelSize;
                auto _y = Parameters::yMin + y * Parameters::voxelSize;
                auto _z = Parameters::zMin + z * Parameters::voxelSize;
                Parameters::Vec3Type pos(_x, _y, _z);
                Belief belief = gaussianProcessMap->belief(pos);

                visualization_msgs::Marker cell;
                cell.id = (int) QVoxel::computeHash(TrueMap::coordToKey(pos));
                double mean = belief.mean();
//                std::cout << mean << std::endl;
//                mean = std::min(1., std::max(0., mean));
//                if (mean < 0.49 && pos.norm() < 1.5 || pos.norm() < 0.6)
//                    // remove belief
//                    cell.action = 2;
//                else
                cell.action = 0;
                cell.type = visualization_msgs::Marker::CUBE;
                cell.header.frame_id = "map";
                cell.scale.x = Parameters::voxelSize;
                cell.scale.y = Parameters::voxelSize;
                cell.scale.z = Parameters::voxelSize;
                cell.color.a = 1.0;
                double r, g, b;
                QTrueVoxel truth = trueMap.query(_x, _y, _z);
                double error = std::abs(belief.mean()
                                        - trueMap.getVoxelMean(truth));
//                    std::cout << error << std::endl;
                double std = std::sqrt(belief.variance()) * GaussianProcessMap::StdDevScalingFactor;
                double inconsistency = error - k * std;
                if (inconsistency > 0)
                {
                    r = std::min(1., inconsistency);
                    g = std::min(1., inconsistency);
                    b = std::min(1., inconsistency);
                }
                else
                    r = g = b = 0;

                cell.color.r = (float) r;
                cell.color.g = (float) g;
                cell.color.b = (float) b;
                cell.pose.position.x = _x;
                cell.pose.position.y = _y;
                cell.pose.position.z = _z;
                cells.markers.push_back(cell);
            }
        }
    }

    std::cout << "Time to query GP map: " << stopWatchVisualizer.elapsed() << std::endl;

    gaussianProcessMapPublisher.publish(cells);

    ros::spinOnce();
}

void Visualizer::publishTrajectory(const Observable *visualizable)
{
    if (!visualizable)
        return;

    auto trajectory = (Trajectory*) visualizable;
    ros::Rate loop_rate(PaintRate);
    loop_rate.sleep();

    stopWatchVisualizer.restart();

    int id = 23984092;

    visualization_msgs::MarkerArray arrows;
    visualization_msgs::Marker clearArrow;
    clearArrow.action = 3; // clear all
    clearArrow.header.frame_id = "map";
    arrows.markers.push_back(clearArrow);
    for (auto &point : *trajectory)
    {
        auto _x = point.position[0];
        auto _y = point.position[1];
        auto _z = point.position[2];
        Parameters::Vec3Type pos(_x, _y, _z);

        visualization_msgs::Marker arrow;
        arrow.id = id++; //(int) QVoxel::computeHash(TrueMap::coordToKey(pos));
        arrow.action = 0;
        arrow.type = visualization_msgs::Marker::ARROW;
        arrow.header.frame_id = "map";
        arrow.scale.x = .3;
        arrow.scale.y = .02;
        arrow.scale.z = .02;
        arrow.color.a = 1;
        arrow.color.r = 1;
        arrow.color.g = 0;
        arrow.color.b = 0;
        arrow.pose.position.x = _x;
        arrow.pose.position.y = _y;
        arrow.pose.position.z = _z;
//        ROS_INFO("Trajectory point %.2f, %.2f, %.2f", _x, _y, _z);
        arrow.pose.orientation.x = point.orientation.x();
        arrow.pose.orientation.y = point.orientation.y();
        arrow.pose.orientation.z = point.orientation.z();
        arrows.markers.push_back(arrow);
    }

    trajectoryPublisher.publish(arrows);
    ROS_INFO("Published %d trajectory arrows.", (int)arrows.markers.size());

    ros::spinOnce();
}

void Visualizer::publishObservation(const Observable *visualizable)
{
    if (!visualizable)
        return;

    auto observation = (Observation*) visualizable;
    ros::Rate loop_rate(PaintRate);
    loop_rate.sleep();

    stopWatchVisualizer.restart();
    int id = 1231;

    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker clearMarker;
    clearMarker.action = 3; // clear all
    clearMarker.header.frame_id = "map";
    markers.markers.push_back(clearMarker);
    for (auto &measurement: observation->measurements())
    {
        auto _x = measurement.sensor.position.x();
        auto _y = measurement.sensor.position.y();
        auto _z = measurement.sensor.position.z();
#if DIMENSIONS == 2
        _z += Parameters::voxelSize;
#endif
        Parameters::Vec3Type pos(_x, _y, _z);

//        visualization_msgs::Marker arrow;
//        arrow.id = (int) QVoxel::computeHash(TrueMap::coordToKey(pos));
//        arrow.action = 0;
//        arrow.type = visualization_msgs::Marker::ARROW;
//        arrow.header.frame_id = "map";
//        arrow.scale.x = measurement.sensor.range;
//        arrow.scale.y = Parameters::voxelSize * .1;
//        arrow.scale.z = Parameters::voxelSize * .1;
//        arrow.color.a = 1;
//        arrow.color.r = 1;
//        arrow.color.g = 1;
//        arrow.color.b = 0;
//        arrow.pose.position.x = _x;
//        arrow.pose.position.y = _y;
//        arrow.pose.position.z = _z;
//        arrow.pose.orientation.x = measurement.sensor.orientation.x();
//        arrow.pose.orientation.y = measurement.sensor.orientation.y();
//        arrow.pose.orientation.z = measurement.sensor.orientation.z();
//        markers.markers.push_back(arrow);


        auto point = measurement.sensor.endPoint();
//        for (auto &point : measurement.sensor.discretized(measurement.value))
        {
//            _x = point.position.x();
//            _y = point.position.y();
//            _z = point.position.z();
            _x = point.x();
            _y = point.y();
            _z = point.z();
#if DIMENSIONS == 2
            _z += Parameters::voxelSize;
#endif
            _x += (rand() * 1. / RAND_MAX - .5) * Parameters::voxelSize;
            _y += (rand() * 1. / RAND_MAX - .5) * Parameters::voxelSize;
            _z += (rand() * 1. / RAND_MAX - .5) * Parameters::voxelSize;
            pos = Parameters::Vec3Type(_x, _y, _z);

            visualization_msgs::Marker dot;
            dot.id = id++; // QVoxel::computeHash(TrueMap::coordToKey(pos));
            dot.action = 0;
            dot.type = visualization_msgs::Marker::SPHERE;
            dot.header.frame_id = "map";
            dot.scale.x = Parameters::voxelSize * .1;
            dot.scale.y = Parameters::voxelSize * .1;
            dot.scale.z = Parameters::voxelSize * .1;
            dot.color.a = 1;
//            if (point.occupied)
            {
                dot.color.r = .1;
                dot.color.g = .2;
                dot.color.b = 0.2;
            }
//            else
//            {
//                continue;
//
//                dot.color.r = .2;
//                dot.color.g = .4;
//                dot.color.b = 1;
//            }
            dot.pose.position.x = _x;
            dot.pose.position.y = _y;
            dot.pose.position.z = _z;
            markers.markers.push_back(dot);
        }
    }

    std::cout << "Publishing " << markers.markers.size() << " markers." << std::endl;
    observationPublisher.publish(markers);

    ros::spinOnce();
}

void Visualizer::sleep(int milliseconds)
{
    ros::Rate loop_rate(milliseconds);
    loop_rate.sleep();
}

double Visualizer::hue2rgb(double arg1, double arg2, double h)
{
    if (h < 0)
        h += 1;
    if (h > 1)
        h -= 1;
    if ((6 * h) < 1)
        return (arg1 + (arg2 - arg1) * 6 * h);
    if ((2 * h) < 1)
        return arg2;
    if ((3 * h) < 2)
        return (arg1 + (arg2 - arg1) * ((2.0 / 3.0) - h) * 6);
    return arg1;
}

void Visualizer::hsl2rgb(double h, double s, double l, double &r, double &g, double &b)
{
    if (s <= 1e-3)
        r = g = b = l;
    else
    {
        double arg1, arg2;
        if (l < 0.5)
            arg2 = l * (1 + s);
        else
            arg2 = (l + s) - (s * l);
        arg1 = 2 * l - arg2;

        r = hue2rgb(arg1, arg2, (h + 1.0 / 3.0));
        g = hue2rgb(arg1, arg2, h);
        b = hue2rgb(arg1, arg2, (h - 1.0 / 3.0));
    }
}
