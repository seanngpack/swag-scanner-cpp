#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "Model.h"
#include "Visualizer.h"
#include "Segmentation.h"
#include "Algorithms.h"


class CalibrationPhysicalFixture : public ::testing::Test {

protected:
    model::Model *mod;
    std::string test_folder_path = "/Users/seanngpack/Programming Stuff/Projects/scanner_files/testing/FileHandlerPhysicalTests";

    virtual void SetUp() {
        mod = new model::Model();
    }

    virtual void TearDown() {
        delete mod;
    }
};


TEST_F(CalibrationPhysicalFixture, TestPlaneCalibration) {

    std::string folder_path = "/Users/seanngpack/Programming Stuff/Projects/scanner_files/18";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>(folder_path + "/filtered/" + "1.pcd", *cloudIn);
    pcl::io::loadPCDFile<pcl::PointXYZ>(folder_path + "/filtered/" + "4.pcd", *cloudOut);
//    std::vector<float> coefs = mod->get_plane_coefs(cloudIn);
//    mod->get_plane_coefs(cloudOut);

    std::vector<float> origin = {-0.0002, 0.0344, 0.4294};
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
    std::vector<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> clouds{cloudOut, transformed};
    visualizer.simpleVis(clouds);

}