#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/bilateral.h>
#include <filesystem>
#include <iostream>
#include "Visualizer.h"
#include "ScanModel.h"
#include "Constants.h"

namespace fs = std::filesystem;

class CompareDepthFilteringFixture : public ::testing::Test {

protected:
    model::ScanModel *mod;
    visual::Visualizer *viewer;

    virtual void SetUp() {
        mod = new model::ScanModel();
        viewer = new visual::Visualizer();
    }

    virtual void TearDown() {
        delete mod;
        delete viewer;
    }
};

/**
 * This test does a four comparison of a scans of the calibration fixture.
 * fixture_1.pcd:
 * "decimation_magnitude": 2,
    "spatial_filter_magnitude": 1,
    "spatial_smooth_alpha": 0.45,
    "spatial_smooth_delta": 5

 *   fixture_2.pcd:
     "spatial_filter_magnitude": 5,

 *  fixture_3.pcd:
 *  "spatial_filter_magnitude": 5,
 *  "spatial_smooth_alpha": 0.6,
 *  This one performs the best tbh
 */

TEST_F(CompareDepthFilteringFixture, CompareCalFixture) {
    using namespace constants;
    std::cout << "Current path is " << fs::current_path() << '\n';
    auto fixture_raw = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto fixture_1 = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto fixture_2 = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto fixture_3 = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::io::loadPCDFile<pcl::PointXYZ>(fs::current_path().string() + "/research/depthFiltering/data/fixture_raw.pcd",
                                        *fixture_raw);
    pcl::io::loadPCDFile<pcl::PointXYZ>(fs::current_path().string() + "/research/depthFiltering/data/fixture_1.pcd",
                                        *fixture_1);
    pcl::io::loadPCDFile<pcl::PointXYZ>(fs::current_path().string() + "/research/depthFiltering/data/fixture_2.pcd",
                                        *fixture_2);
    pcl::io::loadPLYFile<pcl::PointXYZ>("/Users/seanngpack/Desktop/test.ply", *fixture_3);
//    pcl::io::loadPCDFile<pcl::PointXYZ>(fs::current_path().string() + "/research/depthFiltering/data/fixture_3.pcd", *fixture_3);

    fixture_1 = mod->crop_cloud_cpy(fixture_1, cal_min_x, cal_max_x, cal_min_y, cal_max_y, cal_min_z, cal_max_z);

    viewer->compareVisFour(fixture_raw, fixture_1, fixture_2, fixture_3);
}


/**
 * This test compares realsense spatial filtering to pcl bilateral filtering
 * fixture_raw is raw cloud
 *
 * Doesn't seem like bilateral filtering does a very good job,
 *
 * fixture_1 is realsense filtered cloud
 * fixture_2 is pcl filtered cloud
 * fixture_3 is pcl filtered cloud
 */
TEST_F(CompareDepthFilteringFixture, CompareRealsensePCL) {
    using namespace constants;
    std::cout << "Current path is " << fs::current_path() << '\n';
    auto fixture_raw = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto fixture_1 = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto fixture_2 = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto fixture_3 = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::io::loadPCDFile<pcl::PointXYZ>(fs::current_path().string() + "/research/depthFiltering/data/fixture_raw.pcd",
                                        *fixture_raw);
    pcl::io::loadPCDFile<pcl::PointXYZ>(fs::current_path().string() + "/research/depthFiltering/data/fixture_1.pcd",
                                        *fixture_1);

    *fixture_2 = *fixture_raw;
    mod->crop_cloud(fixture_2, cal_min_x, cal_max_x, cal_min_y, cal_max_y, cal_min_z, cal_max_z);
    mod->voxel_grid_filter(fixture_2, .001);
    mod->bilateral_filter(fixture_2, 50, .05);
    *fixture_3 = *fixture_raw;
    mod->bilateral_filter(fixture_3, 1, .01);
    mod->crop_cloud(fixture_3, cal_min_x, cal_max_x, cal_min_y, cal_max_y, cal_min_z, cal_max_z);
    mod->voxel_grid_filter(fixture_3, .001);

    fixture_1 = mod->crop_cloud_cpy(fixture_1, cal_min_x, cal_max_x, cal_min_y, cal_max_y, cal_min_z, cal_max_z);

    viewer->compareVisFour(fixture_raw, fixture_1, fixture_2, fixture_3);
}