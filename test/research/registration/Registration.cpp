#define private public

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <filesystem>
#include <iostream>
#include <pcl/registration/icp.h>
#include <pcl/registration/ia_ransac.h>
#include <vector>
#include <memory>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/filter.h>
#include "Visualizer.h"
#include "ProcessingModel.h"
#include "Normal.h"
#include "Plane.h"
#include "Point.h"
#include "ScanFileHandler.h"
#include "Algorithms.h"
#include "Constants.h"
#include "CalibrationFileHandler.h"
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace fs = std::filesystem;

class RegistrationFixture : public ::testing::Test {

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

/**
 * This method is to test processing and registration algorithms.
 * Will remove processing once i figure out a good processing scheme.
 * Config settings:
 * "decimation_magnitude": 2,
    "spatial_filter_magnitude": 5,
    "spatial_smooth_alpha": 0.6,
    "spatial_smooth_delta": 5
 */

TEST_F(RegistrationFixture, TestRegistration) {
    using namespace constants;

    auto fixture_raw = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto *file_handler = new file::ScanFileHandler();
    auto world_clouds = file_handler->load_clouds(CloudType::Type::FILTERED);

//    file_handler->save_cloud(world_clouds[0], "0.pcd", CloudType::Type::PROCESSED);
//    file_handler->save_cloud(world_clouds[1], "angle.pcd", CloudType::Type::PROCESSED);

    json info_json = file_handler->get_info_json();
    int angle = info_json["angle"];
    std::cout << "degrees per rotation: " << angle << std::endl;

    auto global_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    *global_cloud = *world_clouds[0];
    pcl::PointCloud<pcl::PointXYZ> rotated;
    for (int i = 1; i < world_clouds.size(); i++) {
        rotated = algos::rotate_cloud_about_z_axis(world_clouds[i], angle * i);
        *global_cloud += rotated;
        rotated.clear();
    }
//    viewer->simpleVis(global_cloud);


    auto global_cloud_icp = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto result = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto source = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto target = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto src_to_tgt = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto temp = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    // create global transform and start it at angle deg
    Eigen::Matrix4f global_trans = Eigen::Matrix4f::Identity();
    Eigen::Affine3f rot_trans(Eigen::Affine3f::Identity());
    rot_trans.rotate(Eigen::AngleAxisf(-(angle * M_PI) / 180, Eigen::Vector3f::UnitZ()));
//    global_trans *= rot_trans.matrix();

    Eigen::Matrix4f pair_trans = Eigen::Matrix4f::Identity();
    for (int i = 1; i < world_clouds.size(); i++) {
        target = world_clouds[i - 1];
        source = world_clouds[i];
        *src_to_tgt = algos::rotate_cloud_about_z_axis(source, angle); // rotate target to source

        pair_trans = mod->icp_register_pair_clouds(src_to_tgt, target, temp);
        // verify icp at each step
        std::vector<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> clouds = {source, src_to_tgt};
//        viewer->simpleVis(clouds);
        clouds = {target, temp};
//        viewer->simpleVis(clouds);

        pcl::transformPointCloud(*temp, *result, global_trans);

        // visualize result as red, and original target calibration [0] as white
        clouds = {world_clouds[0], result};
//        viewer->simpleVis(clouds);
        // rotate global trans by angle * i
        // note, rotating in negative direction
        global_trans *= pair_trans;
        global_trans *= rot_trans.matrix();

        *global_cloud_icp += *result;

        // shgows the global calibration beind added
//        viewer->simpleVis(global_cloud_icp);

    }

    viewer->compareVis(global_cloud, global_cloud_icp);

    mod->remove_outliers(global_cloud);
    viewer->simpleVis(global_cloud);

}

TEST_F(RegistrationFixture, TestICPSwag) {
    using namespace constants;
    auto cloud_tgt = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto cloud_src = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto cloud_icp = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
    pcl::io::loadPCDFile<pcl::PointXYZ>(fs::current_path().string() + "/research/registration/data/40.pcd",
                                        *cloud_tgt);
    pcl::io::loadPCDFile<pcl::PointXYZ>(fs::current_path().string() + "/research/registration/data/60.pcd",
                                        *cloud_src);
    equations::Normal rot_axis(0.002451460662859972, -0.8828002989292145, -0.4696775645017624);
    pcl::PointXYZ center_pt(-0.005918797571212053, 0.06167422607541084, 0.42062291502952576);
    cloud_tgt = mod->crop_cloud_cpy(cloud_tgt, cal_min_x, cal_max_x, cal_min_y, cal_max_y, cal_min_z, cal_max_z);
    cloud_src = mod->crop_cloud_cpy(cloud_src, cal_min_x, cal_max_x, cal_min_y, cal_max_y, cal_min_z, cal_max_z);

    algos::transform_cloud_to_world(cloud_tgt, center_pt, rot_axis);
    algos::transform_cloud_to_world(cloud_src, center_pt, rot_axis);

    cloud_tgt = mod->crop_cloud_cpy(cloud_tgt, scan_min_x, scan_max_x, scan_min_y, scan_max_y,
                                scan_min_z, scan_max_z);
    cloud_src = mod->crop_cloud_cpy(cloud_src, scan_min_x, scan_max_x, scan_min_y, scan_max_y,
                                scan_min_z, scan_max_z);

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_tgt, *cloud_tgt, indices);
    std::vector<int> indices2;
    pcl::removeNaNFromPointCloud(*cloud_src, *cloud_src, indices2);
    std::cout << "removed" << std::endl;

    // somewhat align the calibration to within 4 deg of the target 0 calibration
//    *cloud_src = mod->rotate_cloud_about_z_axis(cloud_src, 16);
    *cloud_src = algos::rotate_cloud_about_z_axis(cloud_src, 20);

    std::cout << "rotated" << std::endl;

    // apply translation in x direction and y direction of 1.4cm
//    Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
//    trans(0, 3) = .015;
//    trans(1, 3) = .015;
//    pcl::transformPointCloud (*cloud_src, *cloud_src, trans);
//
//    viewer->simpleVis(cloud_src);
//    std::cout << "transformed" << std::endl;

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud_src);
    icp.setInputTarget(cloud_tgt);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-10);
    icp.setMaxCorrespondenceDistance(.05); // not really sure how this affects results
    icp.setEuclideanFitnessEpsilon(.000001); // big effect
    icp.setRANSACOutlierRejectionThreshold(.000001); // doesn't seem to affect results much
    std::cout << "aligning..." << std::endl;
    icp.align(*cloud_icp);

    if (icp.hasConverged()) {
        std::cout << "\nICP has converged, score is " << icp.getFitnessScore() << std::endl;
        transformation_matrix = icp.getFinalTransformation().cast<double>();
        std::cout << transformation_matrix << std::endl;
    } else {
        PCL_ERROR ("\nICP has not converged.\n");
    }

    std::vector<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> clouds1 = {cloud_tgt, cloud_src};
    std::vector<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> clouds = {cloud_tgt, cloud_icp};
    viewer->simpleVis(clouds1);
    viewer->simpleVis(clouds);
}

TEST_F(RegistrationFixture, ViewRegisteredPCD) {
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::io::loadPCDFile<pcl::PointXYZ>("/Users/seanngpack/Library/Application Support/SwagScanner/scans/lotion/processed/REGISTERED.pcd",
                                        *cloud);
    std::cout << cloud->size() << std::endl;
    viewer->simpleVis(cloud);
}