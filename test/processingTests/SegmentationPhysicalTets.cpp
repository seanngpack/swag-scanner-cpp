#ifndef SWAG_SCANNER_SEGMENTATIONPHYSICALTESTS_CPP
#define SWAG_SCANNER_SEGMENTATIONPHYSICALTESTS_CPP

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "Visualizer.h"
#include "Segmentation.h"


TEST(SegmentationPhysicalTests, TestRemovePlane) {
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


#endif //SWAG_SCANNER_SEGMENTATIONPHYSICALTESTS_CPP