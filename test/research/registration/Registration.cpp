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
#include "Constants.h"
#include "ScanFileHandler.h"
#include "CalibrationFileHandler.h"

namespace fs = std::filesystem;

class RegistrationFixture : public ::testing::Test {

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

TEST_F(RegistrationFixture, NotSureWhatTestThisIsYet) {
    auto fixture_raw = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::io::loadPCDFile<pcl::PointXYZ>(fs::current_path().string() + "/research/registration/data/0.pcd",
                                        *fixture_raw);
    auto *cal_file_handler = new file::CalibrationFileHandler();
    std::vector<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> cropped_clouds;
    std::vector<equations::Plane> upright_planes;
    std::vector<equations::Plane> ground_planes;
    auto clouds = cal_file_handler->load_clouds(CloudType::Type::CALIBRATION);
    for (const auto &cloud : clouds) {
//        viewer->simpleVis(cloud);
//        auto cropped = (mod->crop_cloud(cloud, -.10, .10, -100, .13, -100, .49));
//        cropped_clouds.push_back(cropped);
        std::vector<equations::Plane> coeffs = mod->get_calibration_planes_coefs(cloud, false);
        ground_planes.emplace_back(coeffs[0]);
        upright_planes.emplace_back(coeffs[1]);
    }

    equations::Normal axis_dir = mod->calculate_axis_dir(ground_planes);
    equations::Point center = mod->calculate_center_pt(axis_dir, upright_planes);
    std::cout << axis_dir.A << " " << axis_dir.B << " " << axis_dir.C << std::endl;
    std::cout << center.x << " " << center.y << " " << center.z << std::endl;


//    auto global_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
//    *global_cloud = *cropped_clouds[0];
//    for (int i = 1; i < cropped_clouds.size(); i++) {
//        pcl::PointCloud<pcl::PointXYZ> rotated;
//        rotated = mod->rotate_cloud_about_line(filtered_clouds[i], origin, direction, 20 * i);
//        *global_cloud += rotated;
//    }


    viewer->ptVis(clouds[0], pcl::PointXYZ(center.x, center.y, center.z));


//    viewer->ptVis(fixture_raw, pcl::PointXYZ(-0.018700590463195308,
//                                             0.7106068939489973,
//                                             0.8040320766701974));

//    viewer->compareVisFour(fixture_raw, fixture_1, fixture_2, fixture_3);
}