#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <filesystem>
#include <iostream>
#include "Visualizer.h"
#include "Model.h"
#include "Constants.h"

namespace fs = std::filesystem;

class CompareDepthFilteringFixture : public ::testing::Test {

protected:
    model::Model *mod;
    visual::Visualizer *viewer;

    virtual void SetUp() {
        mod = new model::Model();
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
    pcl::io::loadPCDFile<pcl::PointXYZ>(fs::current_path().string() + "/research/depthFiltering/data/fixture_raw.pcd", *fixture_raw);
    pcl::io::loadPCDFile<pcl::PointXYZ>(fs::current_path().string() + "/research/depthFiltering/data/fixture_1.pcd", *fixture_1);
    pcl::io::loadPCDFile<pcl::PointXYZ>(fs::current_path().string() + "/research/depthFiltering/data/fixture_2.pcd", *fixture_2);
    pcl::io::loadPCDFile<pcl::PointXYZ>(fs::current_path().string() + "/research/depthFiltering/data/fixture_3.pcd", *fixture_3);
    fixture_1 = mod->crop_cloud(fixture_1, min_x, max_x, min_y, max_y, min_z, max_z);

    viewer->compareVisFour(fixture_raw, fixture_1, fixture_2, fixture_3);
}