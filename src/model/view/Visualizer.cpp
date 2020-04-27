#include "Visualizer.h"

using namespace std::chrono_literals;

visual::Visualizer::Visualizer() {

}

//pcl::visualization::PCLVisualizer::Ptr visual::Visualizer::simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
//    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
//    viewer->setBackgroundColor(0, 0, 0);
//    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
//    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
//    viewer->addCoordinateSystem(1.0);
//    viewer->initCameraParameters();
//    return (viewer);
//}

//void visual::Visualizer::start_visualization(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
//    pcl::visualization::PCLVisualizer::Ptr viewer = simpleVis(cloud);
//    while (!viewer->wasStopped()) {
//        viewer->spinOnce(100);
//        std::this_thread::sleep_for(100ms);
//    }
//}


void visual::Visualizer::stop_visualization() {

}
