#ifndef SWAG_SCANNER_SEGMENTATIONPHYSICALTESTS_CPP
#define SWAG_SCANNER_SEGMENTATIONPHYSICALTESTS_CPP

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "Visualizer.h"
#include "Segmentation.h"


TEST(SegmentationPhysicalTests, TestRemovePlane) {
    GTEST_SKIP();
    std::string test_folder_path = "/Users/seanngpack/Programming Stuff/Projects/scanner_files/testing/FileHandlerPhysicalTests";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInSegmented(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutSegmented(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::io::loadPCDFile<pcl::PointXYZ>(test_folder_path + "/raw/" + "1.pcd", *cloudIn);
    pcl::io::loadPCDFile<pcl::PointXYZ>(test_folder_path + "/raw/" + "2.pcd", *cloudOut);


    cloudInSegmented = segmentation::remove_plane(cloudIn);
    cloudOutSegmented = segmentation::remove_plane(cloudOut);
    std::cout << cloudInSegmented->size() << std::endl;
    std::cout << cloudInSegmented->points[100] << std::endl;

//    visual::Visualizer visualizer;
//    visualizer.simpleVis(cloudInSegmented);
}

TEST(SegmentationPhysicalTests, TestGetPlanes) {
    std::string folder_path = "/Users/seanngpack/Library/Application Support/SwagScanner/calibration/fish_cup";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>(folder_path + "/10.pcd", *cloudIn);
    segmentation::get_calibration_planes_coefs(cloudIn);

//    visual::Visualizer *viewer;
//    viewer->simpleVis(cloudIn);
}

// view rotation axis on the visualizer
// WARNING: This test is outdated!! Update with latest scan info
TEST(SegmentationPhysicalTests, ViewAxis) {
    using namespace std::chrono_literals;
    std::string folder_path = "/Users/seanngpack/Library/Application Support/SwagScanner/calibration";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>(folder_path + "/processed_1/15.pcd", *cloudIn);

    pcl::PointXYZ p1;
    p1.x = -0.0002;
    p1.y = 0.0344;
    p1.z = 0.4294;


    pcl::PointXYZ p2;
    p2.x = -0.2867;
    p2.y = -14.6891;
    p2.z = -8.0644;



    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloudIn, "sample cloud");
    viewer->addLine(p1, p2, std::string("line"), 0);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }
}


#endif //SWAG_SCANNER_SEGMENTATIONPHYSICALTESTS_CPP