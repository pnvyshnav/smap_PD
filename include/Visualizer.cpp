#include "Visualizer.h"
#include "TrueMap.h"
#include "BeliefMap.h"
#include "Sensor.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/GridCells.h>

void Visualizer::render()
{
    ROS_INFO("Render");
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::spin();
}

Visualizer::Visualizer(int argc, char **argv)
{
    ros::init(argc, argv, "SMAP");
    nodeHandle = new ros::NodeHandle;
    //trueMapPublisher = nodeHandle->advertise<visualization_msgs::Marker>("true_map", 1000);
    trueMap2dPublisher = nodeHandle->advertise<nav_msgs::GridCells>("true_map_2d", 0);
    beliefMapPublisher = nodeHandle->advertise<visualization_msgs::MarkerArray>("belief_map", 1000);
    sensorPublisher = nodeHandle->advertise<visualization_msgs::Marker>("sensor", 1000);
    rayVoxelPublisher = nodeHandle->advertise<visualization_msgs::MarkerArray>("ray_voxels", 1000);
}

Visualizer::~Visualizer()
{
    delete nodeHandle;
}

void Visualizer::publishTrueMap(const Visualizable *visualizable)
{
    auto trueMap = (TrueMap*) visualizable;
    ROS_INFO("Publishing TrueMap ....");
    ros::Rate loop_rate(10);
    int step = 0;

    while (ros::ok())
    {
        loop_rate.sleep();

        nav_msgs::GridCells grid;
        grid.header.frame_id = "map";
        grid.cell_height = Parameters::voxelSize;
        grid.cell_width = Parameters::voxelSize;

        for (unsigned int x = 0; x < Parameters::voxelsPerDimensionX; ++x)
        {
            for (unsigned int y = 0; y < Parameters::voxelsPerDimensionY; ++y)
            {
                for (unsigned int z = 0; z < Parameters::voxelsPerDimensionZ; ++z)
                {
                    auto _x = Parameters::xMin + x * Parameters::voxelSize;
                    auto _y = Parameters::yMin + y * Parameters::voxelSize;
                    auto _z = Parameters::zMin + z * Parameters::voxelSize;
                    QTrueVoxel voxel = trueMap->query(_x, _y, _z);
                    if (voxel.node->getOccupancy() < 0.5)
                        continue;

                    geometry_msgs::Point p;
                    p.x = _x;
                    p.y = _y;
                    p.z = _z;

                    grid.cells.push_back(p);
                }
            }
        }


        trueMap2dPublisher.publish(grid);

        ros::spinOnce();
        ++step;
        if (step > 10)
            break;
    }
}

void Visualizer::publishBeliefMap(const Visualizable *visualizable)
{
    auto beliefMap = (BeliefMap*) visualizable;
    ROS_INFO("Publishing BeliefMap ....");
    ros::Rate loop_rate(10);

    int step = 0;
    while (ros::ok())
    {
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
                    QBeliefVoxel voxel = beliefMap->query(_x, _y, _z);

                    visualization_msgs::Marker cell;
                    cell.action = 0;
                    cell.id = (int) voxel.hash;
                    cell.type = visualization_msgs::Marker::CUBE;
                    cell.header.frame_id = "map";
                    cell.scale.x = Parameters::voxelSize;
                    cell.scale.y = Parameters::voxelSize;
                    cell.scale.z = Parameters::voxelSize;
                    cell.color.a = 0.9;
                    float intensity = (float) (1.0 - voxel.node->getValue()->mean());
                    cell.color.r = intensity;
                    cell.color.g = intensity;
                    cell.color.b = intensity;
                    cell.pose.position.x = _x;
                    cell.pose.position.y = _y;
                    cell.pose.position.z = _z;
                    cells.markers.push_back(cell);
                }
            }
        }

        beliefMapPublisher.publish(cells);

        // Publish ICM's ray voxels
        visualization_msgs::MarkerArray rayVoxels;
        visualization_msgs::Marker clearVoxel;
        clearVoxel.action = 3; // clear all
        clearVoxel.header.frame_id = "map";
        rayVoxels.markers.push_back(clearVoxel);
        if (beliefMap->icm != NULL)
        {
            unsigned int i = 0;
            for (auto &key : beliefMap->icm->ray)
            {
                QBeliefVoxel beliefVoxel = beliefMap->query(key);

                visualization_msgs::Marker rayVoxel;
                rayVoxel.action = 0;
                rayVoxel.id = (int) beliefVoxel.hash;
                rayVoxel.type = visualization_msgs::Marker::CUBE;
                rayVoxel.header.frame_id = "map";
                rayVoxel.scale.x = Parameters::voxelSize;
                rayVoxel.scale.y = Parameters::voxelSize;
                rayVoxel.scale.z = Parameters::voxelSize;
                rayVoxel.color.a = 0.7;

                if (beliefVoxel.type != GEOMETRY_VOXEL)
                {
                    rayVoxel.color.r = 0.2;
                    rayVoxel.color.g = 0.2;
                    rayVoxel.color.b = 0.2;
                }
                else
                {
                    rayVoxel.color.r = 0.9;
                    rayVoxel.color.g = 0.4;
                    rayVoxel.color.b = 0.0;
                }

                rayVoxel.pose.position.x = beliefVoxel.position.x();
                rayVoxel.pose.position.y = beliefVoxel.position.y();
                rayVoxel.pose.position.z = beliefVoxel.position.z();
                rayVoxels.markers.push_back(rayVoxel);
                i += 1;
                if (i >= beliefMap->icm->rayLength)
                    break;
            }
        }

        rayVoxelPublisher.publish(rayVoxels);

        ros::spinOnce();
        ++step;
        if (step > 0)
            break;
    }
}

void Visualizer::publishSensor(const Visualizable *visualizable)
{
    auto sensor = (Sensor*) visualizable;
    ROS_INFO("Publishing Sensor ....");
    ros::Rate loop_rate(10);
    int i = 0;

    while (ros::ok())
    {
        loop_rate.sleep();

        visualization_msgs::Marker arrow;
        arrow.action = 0;
        arrow.id = (int) visualizable->visualizationId();
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
        base.z = sensor->position().z() + 0.1;
        arrow.points.push_back(base);
        geometry_msgs::Point target;
        target.x = sensor->position().x() + Parameters::sensorRange * sensor->orientation().x();
        target.y = sensor->position().y() + Parameters::sensorRange * sensor->orientation().y();
        target.z = sensor->position().z() + Parameters::sensorRange * sensor->orientation().z() + 0.1;
        arrow.points.push_back(target);

        sensorPublisher.publish(arrow);

        ros::spinOnce();
        ++i;
        if (i > 0)
            break;
    }
}

void Visualizer::publishRay(TrueMap &trueMap, Sensor &sensor)
{
    octomap::KeyRay ray;
    trueMap.computeRayKeys(sensor.position(), sensor.position() + sensor.orientation() * Parameters::sensorRange, ray);
    ROS_INFO_STREAM("PUBLISHING SENSOR " << sensor.position() << " to " << (sensor.position() + sensor.orientation() * Parameters::sensorRange));

    ROS_INFO("Publishing SENSOR ....");
    ros::Rate loop_rate(10);

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
            rayVoxel.color.a = 0.9;
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
