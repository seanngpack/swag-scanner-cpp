#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
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

namespace fs = std::filesystem;

class SegmentationFixture : public ::testing::Test {

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

 * This test is to test calibration plane segmentation.
 */
TEST_F(SegmentationFixture, TestCalibrationPlaneSegmentation) {
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto cloud_plane = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto cloud_normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();

    pcl::io::loadPCDFile<pcl::PointXYZ>(fs::current_path().string() + "/research/segmentation/data/0.pcd", *cloud);;
    cloud = (mod->crop_cloud(cloud, -.10, .10, -100, .11, -100, .49));

    // calculate the normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud);
    ne.setKSearch(50);
    ne.compute(*cloud_normals);

    auto ground_coeff = std::make_shared<pcl::ModelCoefficients>();
    auto inliers = std::make_shared<pcl::PointIndices>();
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PARALLEL_PLANE);
    seg.setNormalDistanceWeight(0.02);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(10000);
    seg.setDistanceThreshold(0.005);
    seg.setInputCloud(cloud);
    seg.setInputNormals(cloud_normals);
    // set hardcoded ground normal axis value with wide epsilon value
    seg.setAxis(Eigen::Vector3f(.00295, -.7803, -.3831));
    seg.setEpsAngle(0.523599);
    seg.segment(*inliers, *ground_coeff);

    if (inliers->indices.empty()) {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    }

    std::cerr << "Model coefficients: " << ground_coeff->values[0] << " "
              << ground_coeff->values[1] << " "
              << ground_coeff->values[2] << " "
              << ground_coeff->values[3] << std::endl;

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);

    // Get the points associated with the planar surface
    extract.filter(*cloud_plane);

    visual::Visualizer visualizer;
    std::vector<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> clouds{cloud, cloud_plane};
    visualizer.simpleVis(clouds);

    // Remove the planar inliers, extract the rest
    extract.setNegative(true);
    extract.filter(*cloud);

    // remove normal inliers
    extract_normals.setNegative(true);
    extract_normals.setInputCloud(cloud_normals);
    extract_normals.setIndices(inliers);
    extract_normals.filter(*cloud_normals);

    viewer->normalsVis(cloud, cloud_normals);

    // LETS GET THE UPRIGHT PLANE!!

    auto up_coeff = std::make_shared<pcl::ModelCoefficients>();
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg2;

    seg2.setOptimizeCoefficients(true);
    seg2.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg2.setNormalDistanceWeight(0.02);
    seg2.setMethodType(pcl::SAC_RANSAC);
    seg2.setMaxIterations(10000);
    seg2.setDistanceThreshold(0.003);
    seg2.setInputCloud(cloud);
    seg2.setInputNormals(cloud_normals);
    seg2.segment(*inliers, *up_coeff);

    std::cerr << "Up Model coefficients: " << up_coeff->values[0] << " "
              << up_coeff->values[1] << " "
              << up_coeff->values[2] << " "
              << up_coeff->values[3] << std::endl;

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);

    // Get the points associated with the planar surface
    extract.filter(*cloud_plane);


    clouds = {cloud, cloud_plane};
    visualizer.simpleVis(clouds);

    auto ground_vect = Eigen::Vector3f(ground_coeff->values[0], ground_coeff->values[1],ground_coeff->values[2]);
    auto up_vect = Eigen::Vector3f(up_coeff->values[0], up_coeff->values[1], up_coeff->values[2]);

    double angle = std::atan2(ground_vect.cross(up_vect).norm(), ground_vect.dot(up_vect));
    double angle_deg = angle * (180.0/3.141592653589793238463);
    std::cout << "the angle between two planes is " << angle_deg << std::endl;
    std::cout << "the error is: " << abs((angle_deg-90)/90.0) * 100.0 << "%" << std::endl;
}