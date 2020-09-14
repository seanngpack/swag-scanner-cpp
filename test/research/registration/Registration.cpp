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
#include "Constants.h"
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

TEST_F(CalibrationFixture, NotSureWhatTestThisIsYet) {
    using namespace constants;

    auto fixture_raw = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::io::loadPCDFile<pcl::PointXYZ>(fs::current_path().string() + "/research/registration/data/0.pcd",
                                        *fixture_raw);
    auto *file_handler = new file::ScanFileHandler();
    equations::Normal rot_axis(0.002451460662859972, -0.8828002989292145,-0.4696775645017624);
    pcl::PointXYZ center_pt(-0.005918797571212053,0.06167422607541084,0.42062291502952576);
    std::vector<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> cropped_clouds;
    std::vector<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> world_clouds;
    auto clouds = file_handler->load_clouds(CloudType::Type::FILTERED);
    for (const auto &c :clouds) {
        cropped_clouds.push_back(mod->crop_cloud(c, cal_min_x, cal_max_x, cal_min_y, cal_max_y, cal_min_z, cal_max_z));
    }

    for (const auto&c : cropped_clouds) {
        auto transformed = mod->transform_cloud_to_world(c, center_pt, rot_axis);
        auto transformed_cropped = mod->crop_cloud(transformed, scan_min_x, scan_max_x, scan_min_y, scan_max_y, scan_min_z, scan_max_z);
        world_clouds.push_back(transformed_cropped);
        viewer->simpleVis(transformed_cropped);
    }



//    auto global_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
//    *global_cloud = *cropped_clouds[0];
//    for (int i = 1; i < cropped_clouds.size(); i++) {
//        pcl::PointCloud<pcl::PointXYZ> rotated;
//        rotated = mod->rotate_cloud_about_line(filtered_clouds[i], origin, direction, 20 * i);
//        *global_cloud += rotated;
//    }

}