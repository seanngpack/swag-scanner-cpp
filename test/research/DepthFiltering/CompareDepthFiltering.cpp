#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <filesystem>
#include <iostream>
#include "Visualizer.h"
#include "Model.h"

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
 * This test does a side-by-side of a scans of the calibration fixture. The filtered scan has default values:
 * "decimation_magnitude": 2,
    "spatial_filter_magnitude": 1,
    "spatial_smooth_alpha": 0.45,
    "spatial_smooth_delta": 5
 */
TEST_F(CompareDepthFilteringFixture, CompareCalFixture) {
    std::cout << "Current path is " << fs::current_path() << '\n';
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_raw(new pcl::PointCloud<pcl::PointXYZ>);
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>(fs::current_path().string() + "/research/DepthFiltering/data/fixture_default_raw.pcd", *cloud_raw);
    pcl::io::loadPCDFile<pcl::PointXYZ>(fs::current_path().string() + "/research/DepthFiltering/data/fixture_default_filtered.pcd", *cloud_filtered);
    viewer->compareVis(cloud_raw, cloud_filtered);
}