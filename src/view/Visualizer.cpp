#include "Visualizer.h"
#include <algorithm>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

using namespace std::chrono_literals;

visual::Visualizer::Visualizer() {}

void visual::Visualizer::simpleVis(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud) {

    pcl::visualization::PCLVisualizer viewer("3D viewer");
    viewer.setBackgroundColor(0, 0, 0);
    viewer.addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer.addCoordinateSystem(1.0);
    viewer.initCameraParameters();
    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }
}

void visual::Visualizer::simpleVis(const std::vector<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> &clouds) {
    pcl::visualization::PCLVisualizer viewer("3D viewer");

    int r = 255;
    int delta = clouds.size() / 10;

    viewer.addPointCloud<pcl::PointXYZ>(clouds[0], std::to_string(0));
    for (int i = 1; i < clouds.size(); i++) {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(clouds[i], r, 0, 0);
        viewer.addPointCloud<pcl::PointXYZ>(clouds[i], color_handler, std::to_string(i));
        r -= delta; // doesn't scale nicely but that's okay for now'
    }

//    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer.setBackgroundColor(0, 0, 0);
    viewer.addCoordinateSystem(1.0);
    viewer.initCameraParameters();
    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }
}

void visual::Visualizer::simpleVisColor(const std::vector<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> &clouds) {
    pcl::visualization::PCLVisualizer viewer("3D viewer");

    int b = 255;
    int delta = clouds.size() / 10;

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler(clouds[0], 0, 255, 0);
    viewer.addPointCloud<pcl::PointXYZ>(clouds[0], handler, std::to_string(0));
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 9, "0");
    for (int i = 1; i < clouds.size(); i++) {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(clouds[i], 0, 0, b);
        viewer.addPointCloud<pcl::PointXYZ>(clouds[i], color_handler, std::to_string(i));
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 9, std::to_string(i));
        b -= delta; // doesn't scale nicely but that's okay for now'
    }

//    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer.setBackgroundColor(255, 255, 255);
    viewer.addCoordinateSystem(1.0);
    viewer.initCameraParameters();
    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }
}

void visual::Visualizer::ptVis(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud, const pcl::PointXYZ &pt) {
    pcl::visualization::PCLVisualizer viewer("3D viewer");
    auto point = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    point->push_back(pt);

    viewer.setBackgroundColor(0, 0, 0);
    viewer.addCoordinateSystem(1.0);
    viewer.initCameraParameters();

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(point, 255, 0, 0);

    viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud");
    viewer.addPointCloud<pcl::PointXYZ>(point, color_handler, "point");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "point");
    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }
}

void visual::Visualizer::compareVis(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud1,
                                    const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud2) {
    pcl::visualization::PCLVisualizer viewer("3D viewer");
    viewer.initCameraParameters();

    int v1(0);
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer.setBackgroundColor(0, 0, 0, v1);
    viewer.addText("unfiltered cloud", 10, 10, "v1 text", v1);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler1(cloud1, 255, 255, 255);
    viewer.addPointCloud<pcl::PointXYZ>(cloud1, handler1, "sample cloud1", v1);

    int v2(0);
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer.setBackgroundColor(0.3, 0.3, 0.3, v2);
    viewer.addText("filtered cloud with decimation, spatial & termporal", 10, 10, "v2 text", v2);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler2(cloud2, 255, 255, 255);
    viewer.addPointCloud<pcl::PointXYZ>(cloud2, handler2, "sample cloud2", v2);

    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
//    viewer->addCoordinateSystem(1.0);

    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }
}

void visual::Visualizer::normalsVis(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud,
                                    std::shared_ptr<pcl::PointCloud<pcl::Normal>> &normal) {
    pcl::visualization::PCLVisualizer viewer("3D viewer");
    viewer.setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudGeometryHandlerXYZ<pcl::PointXYZ> rgb(cloud);
    viewer.addPointCloud<pcl::PointXYZ>(cloud, rgb, "sample cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normal, 10, 0.05, "normals");
    viewer.addCoordinateSystem(1.0);
    viewer.initCameraParameters();
    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }
}



