// http://pointclouds.org/documentation/tutorials/pcl_visualizer.php
#include "Visualizer.h"

using namespace std::chrono_literals;

visual::Visualizer::Visualizer() {}

pcl::visualization::PCLVisualizer::Ptr visual::Visualizer::simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
//     --------------------------------------------
//     -----Open 3D viewer and add point cloud-----
//     --------------------------------------------
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    while (!view_instance->wasStopped()) {
        view_instance->spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }
    return (viewer);
}

pcl::visualization::PCLVisualizer::Ptr visual::Visualizer::normalsVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                                                                      pcl::PointCloud<pcl::Normal>::ConstPtr normals) {
    // --------------------------------------------------------
    // -----Open 3D viewer and add point cloud and normals-----
    // --------------------------------------------------------
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudGeometryHandlerXYZ<pcl::PointXYZ> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 10, 0.05, "normals");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }
    return (viewer);
}



