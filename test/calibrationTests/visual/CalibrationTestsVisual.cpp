#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "Algorithms.h"
#include "Visualizer.h"


// This shows visually rotating a calibration about a line.
// This test is currently not working because the folder does not exist.
TEST(CalibrationTestsVisual, VisualizePlaneCalibration) {
    std::string folder_path = "/Users/seanngpack/Programming Stuff/Projects/scanner_files/18";
    auto cloudIn = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto cloudOut = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto transformed = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::io::loadPCDFile<pcl::PointXYZ>(folder_path + "/filtered/" + "1.pcd", *cloudIn);
    pcl::io::loadPCDFile<pcl::PointXYZ>(folder_path + "/filtered/" + "4.pcd", *cloudOut);
//    std::vector<float> coefs = mod->get_plane_coefs(cloudIn);
//    mod->get_plane_coefs(cloudOut);

    std::vector<float> origin = {-0.000213082, 0.0298714, 0.42673};
    std::vector<float> direction = {-0.0158, -0.8661, -0.4996};

    // from 1.pcd -> 4.pcd
    float theta = 0.15708;

    transformed->resize(cloudIn->size());

    for (int i = 0; i < cloudIn->size(); i++) {
        transformed->points[i] = algos::rotate_point_about_line(cloudIn->points[i],
                                                                origin,
                                                                direction,
                                                                theta);
    }


    visual::Visualizer visualizer;
    std::vector<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> clouds{cloudOut, transformed};
    visualizer.simpleVis(clouds);

}