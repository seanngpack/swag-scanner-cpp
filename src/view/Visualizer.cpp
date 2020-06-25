#include "Visualizer.h"

using namespace std::chrono_literals;

visual::Visualizer::Visualizer() {}

void visual::Visualizer::simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }
}

void visual::Visualizer::simpleVis(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ>::Ptr>> clouds) {
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

    int r = 255;
    int delta = clouds.size() / 10;

    viewer->addPointCloud<pcl::PointXYZ>(clouds[0], std::to_string(0));
    for (int i = 1; i < clouds.size(); i++) {
        pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZ> color_handler(clouds[i], r, 0, 0);
        viewer->addPointCloud<pcl::PointXYZ>(clouds[i], color_handler, std::to_string(i));
        r -= delta; // doesn't scale nicely but that's okay for now'
    }
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }
}

void visual::Visualizer::simpleVis(std::vector<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> clouds) {
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

    int r = 255;
    int delta = clouds.size() / 10;

    viewer->addPointCloud<pcl::PointXYZ>(clouds[0], std::to_string(0));
    for (int i = 1; i < clouds.size(); i++) {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(clouds[i], r, 0, 0);
        viewer->addPointCloud<pcl::PointXYZ>(clouds[i], color_handler, std::to_string(i));
        r -= delta; // doesn't scale nicely but that's okay for now'
    }

//    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }
}

void visual::Visualizer::normalsVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
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
}



