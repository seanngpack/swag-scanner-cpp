#include "ProcessingModel.h"
#include "Visualizer.h"
#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <filesystem>

namespace fs = std::filesystem;

class ConvexHullFixture : public ::testing::Test {

protected:
    model::ProcessingModel *mod;
    visual::Visualizer *viewer;

    virtual void SetUp() {
        mod = new model::ProcessingModel();
        viewer = new visual::Visualizer();
    }

    virtual void TearDown() {
        delete mod;
        delete viewer;
    }
};

TEST_F(ConvexHullFixture, VisualizeConvex) {
    auto rabbit_registered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::io::loadPCDFile<pcl::PointXYZ>(fs::current_path().string() + "/research/meshing/data/rabbit_registered.pcd",
                                        *rabbit_registered);
    // perform concave hull
    // visualize!
}
