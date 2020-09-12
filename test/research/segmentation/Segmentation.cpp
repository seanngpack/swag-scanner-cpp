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

    pcl::io::loadPCDFile<pcl::PointXYZ>(fs::current_path().string() + "/research/segmentation/data/20.pcd", *cloud);;
    cloud = (mod->crop_cloud(cloud, -.10, .10, -100, .11, -100, .49));

    // calculate the normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud);
    ne.setKSearch(50);
    ne.compute(*cloud_normals);

    auto coefficients = std::make_shared<pcl::ModelCoefficients>();
    auto inliers = std::make_shared<pcl::PointIndices>();
    // Create the segmentation object
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_NORMAL_PARALLEL_PLANE);
    seg.setNormalDistanceWeight(0.08);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(10000);
    seg.setDistanceThreshold(0.1);
    seg.setInputCloud(cloud);
    seg.setInputNormals(cloud_normals);
    seg.setAxis(Eigen::Vector3f(.00295, -.7803, -.3831));
    seg.setEpsAngle(0.523599);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0) {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");

    }

    std::cerr << "Model coefficients: " << coefficients->values[0] << " "
              << coefficients->values[1] << " "
              << coefficients->values[2] << " "
              << coefficients->values[3] << std::endl;

    //  -0.0320766 -0.877187 -0.479076 0.254496

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

    // everything below we remove the other plane
    // Remove the planar inliers, extract the rest
    extract.setNegative(true);
    extract.filter(*cloud);

    extract_normals.setNegative(true);
    extract_normals.setInputCloud(cloud_normals);
    extract_normals.setIndices(inliers);
    extract_normals.filter(*cloud_normals);


//    viewer->normalsVis(cloud, cloud_normals);


    pcl::SACSegmentation<pcl::PointXYZ> seg2;
    // Optional
    seg2.setOptimizeCoefficients(true);
    // Mandatory
    seg2.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg2.setMethodType(pcl::SAC_RANSAC);
    seg2.setMaxIterations(10000);
    seg2.setDistanceThreshold(0.1);
    seg2.setAxis(Eigen::Vector3f(coefficients->values[0], coefficients->values[1],
                                 coefficients->values[2]));
    seg2.setEpsAngle(0.2);
    std::cerr << seg2.getAxis() << std::endl;
    seg2.setInputCloud(cloud);
    seg2.segment(*inliers, *coefficients);

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);

    // Get the points associated with the planar surface
    extract.filter(*cloud_plane);

    visualizer.simpleVis(clouds);

    std::cerr << "Model coefficients: " << coefficients->values[0] << " "
              << coefficients->values[1] << " "
              << coefficients->values[2] << " "
              << coefficients->values[3] << std::endl;


    seg.setModelType(pcl::SACMODEL_NORMAL_PARALLEL_PLANE);
    seg.setAxis(Eigen::Vector3f(coefficients->values[0], coefficients->values[1],
                                 coefficients->values[2]));
    std::cerr << seg.getAxis() << std::endl;
    seg.segment(*inliers, *coefficients);

    std::cerr << "Model coefficients: " << coefficients->values[0] << " "
              << coefficients->values[1] << " "
              << coefficients->values[2] << " "
              << coefficients->values[3] << std::endl;

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);

    // Get the points associated with the planar surface
    extract.filter(*cloud_plane);


    visualizer.simpleVis(clouds);
}