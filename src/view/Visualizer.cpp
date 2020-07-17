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

void visual::Visualizer::compareVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud1,
                                    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud2) {
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->initCameraParameters();

    int v1(0);
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor(0, 0, 0, v1);
    viewer->addText("unfiltered cloud", 10, 10, "v1 text", v1);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler1(cloud1, 0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud1, handler1, "sample cloud1", v1);

    int v2(0);
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
    viewer->addText("filtered cloud with decimation, spatial & termporal", 10, 10, "v2 text", v2);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler2(cloud2, 0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud2, handler2, "sample cloud2", v2);

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
    viewer->addCoordinateSystem(1.0);

    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }

}



