#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <filesystem>
#include <iostream>
#include <vector>
#include <memory>
#include "Visualizer.h"
#include "Model.h"
#include "Normal.h"
#include "Plane.h"
#include "Point.h"
#include "ScanFileHandler.h"
#include "Algorithms.h"
#include "CalibrationFileHandler.h"

namespace fs = std::filesystem;

class CalibrationFixture : public ::testing::Test {

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
 * Config settings:
 * "decimation_magnitude": 2,
    "spatial_filter_magnitude": 5,
    "spatial_smooth_alpha": 0.6,
    "spatial_smooth_delta": 5
 */
TEST_F(CalibrationFixture, CalibrationTests) {
    auto fixture_raw = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::io::loadPCDFile<pcl::PointXYZ>(fs::current_path().string() + "/research/registration/data/0.pcd",
                                        *fixture_raw);
    auto *cal_file_handler = new file::CalibrationFileHandler();
    std::vector<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> cropped_clouds;
    std::vector<equations::Plane> upright_planes;
    std::vector<equations::Plane> ground_planes;
    auto clouds = cal_file_handler->load_clouds(CloudType::Type::CALIBRATION);
    for (const auto &cloud : clouds) {
        std::vector<equations::Plane> coeffs = mod->get_calibration_planes_coefs(cloud, true);
        ground_planes.emplace_back(coeffs[0]);
        upright_planes.emplace_back(coeffs[1]);
    }

    equations::Normal axis_dir = mod->calculate_axis_dir(ground_planes);
    pcl::PointXYZ center = mod->calculate_center_pt(axis_dir, upright_planes);
    std::cout << axis_dir.A << " " << axis_dir.B << " " << axis_dir.C << std::endl;
    std::cout << "found point" << center.x << " " << center.y << " " << center.z << std::endl;

    pcl::PointXYZ projected_pt;
    projected_pt = algos::project_point_to_plane(center,
                                                 algos::find_point_in_plane(clouds[0], ground_planes[0], .00001),
                                                 axis_dir);

    std::cout << "new projected point" << projected_pt << std::endl;

    viewer->ptVis(clouds[0], center);
    viewer->ptVis(clouds[0], projected_pt);
}