#include "../include/PointCloud.h"

#include <ros/ros.h>

#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>

void PointCloud::visualize() const
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(1, 1, 1);
    pcl::PointCloud<Parameters::PointType>::ConstPtr cloud(new pcl::PointCloud<Parameters::PointType>(_cloud));
    viewer->addPointCloud<Parameters::PointType>(cloud);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0., 0., 0.);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}

void PointCloud::loadPly(std::string filename)
{
    ROS_INFO_STREAM("Reading point cloud from " << filename << "...");
    std::ifstream fin(filename);
    std::string linestr;
    for (unsigned int skipHeaderLines = 0; skipHeaderLines < 11; ++skipHeaderLines)
        std::getline(fin, linestr); // header row - throw away
    int count = 0;
    float minx = 0, miny = 0, minz = 0, maxx = 0, maxy = 0, maxz = 0;
    for (std::string line; std::getline(fin, line); ++count)
    {
        Parameters::PointType point;
        std::istringstream iss(line);
        iss >> point.x;
        iss >> point.y;
        iss >> point.z;
        iss >> point.intensity;
        _cloud.points.push_back(point);
        minx = std::min(minx, point.x);
        miny = std::min(miny, point.y);
        minz = std::min(minz, point.z);
        maxx = std::max(maxx, point.x);
        maxy = std::max(maxy, point.y);
        maxz = std::max(maxz, point.z);

        // skip some points
        for (int skip = 0; std::getline(fin, line) && skip < 2; ++skip);
    }

    if (count == 0)
    {
        ROS_WARN("Could not read any points!");
    }
    else
    {
        ROS_INFO("Read %d points.", count);
        ROS_INFO("Point cloud range: (%.3f, %.3f, %.3f) - (%.3f, %.3f, %.3f)",
                 minx, miny, minz, maxx, maxy, maxz);
    }
}
