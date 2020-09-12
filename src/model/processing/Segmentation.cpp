#include "Segmentation.h"
#include "Visualizer.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>


std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>
segmentation::remove_plane(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud) {
    pcl::ModelCoefficients coefficients;
    auto inliers = std::make_shared<pcl::PointIndices>();
    auto cloud_inliers = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto cloud_outliers = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    coefficients.values.resize(4);
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.005);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, coefficients);

    if (inliers->indices.empty()) {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    }

    std::cerr << "Model coefficients: " << coefficients.values[0] << " "
              << coefficients.values[1] << " "
              << coefficients.values[2] << " "
              << coefficients.values[3] << std::endl;


    // Extract inliers
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);            // Extract the inliers
    extract.filter(*cloud_inliers);        // cloud_inliers contains the plane

    // Extract outliers
    extract.setNegative(true);                // Extract the outliers
    extract.filter(*cloud_outliers);        // cloud_outliers contains everything but the plane
    return cloud_outliers;
}

std::vector<equations::Plane>
segmentation::get_calibration_planes_coefs(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud,
                                           bool visual_flag) {
    visual::Visualizer *viewer = nullptr;
    std::vector<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> clouds;
    if (visual_flag) {
        viewer = new visual::Visualizer();
    }
    std::vector<equations::Plane> planes;
    auto cloud_cpy = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto cloud_plane = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto cloud_normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();

    // calculate the normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud_cpy);
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
    seg.setInputCloud(cloud_cpy);
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
    extract.setInputCloud(cloud_cpy);
    extract.setIndices(inliers);
    extract.setNegative(false);

    // Get the points associated with the planar surface
    extract.filter(*cloud_plane);

    if (visual_flag) {
        clouds = {cloud_cpy, cloud_plane};
        viewer->simpleVis(clouds);
    }

    // Remove the planar inliers, extract the rest
    extract.setNegative(true);
    extract.filter(*cloud_cpy);

    // remove normal inliers
    extract_normals.setNegative(true);
    extract_normals.setInputCloud(cloud_normals);
    extract_normals.setIndices(inliers);
    extract_normals.filter(*cloud_normals);

    viewer->normalsVis(cloud_cpy, cloud_normals);

    // LETS GET THE UPRIGHT PLANE!!

    auto up_coeff = std::make_shared<pcl::ModelCoefficients>();
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg2;

    seg2.setOptimizeCoefficients(true);
    seg2.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg2.setNormalDistanceWeight(0.02);
    seg2.setMethodType(pcl::SAC_RANSAC);
    seg2.setMaxIterations(10000);
    seg2.setDistanceThreshold(0.003);
    seg2.setInputCloud(cloud_cpy);
    seg2.setInputNormals(cloud_normals);
    seg2.segment(*inliers, *up_coeff);

    std::cerr << "Up Model coefficients: " << up_coeff->values[0] << " "
              << up_coeff->values[1] << " "
              << up_coeff->values[2] << " "
              << up_coeff->values[3] << std::endl;

    extract.setInputCloud(cloud_cpy);
    extract.setIndices(inliers);
    extract.setNegative(false);

    // Get the points associated with the planar surface
    extract.filter(*cloud_plane);

    if (visual_flag) {
        clouds = {cloud_cpy, cloud_plane};
        viewer->simpleVis(clouds);
    }

    auto ground_vect = Eigen::Vector3f(ground_coeff->values[0], ground_coeff->values[1], ground_coeff->values[2]);
    auto up_vect = Eigen::Vector3f(up_coeff->values[0], up_coeff->values[1], up_coeff->values[2]);

    double angle = std::atan2(ground_vect.cross(up_vect).norm(), ground_vect.dot(up_vect));
    double angle_deg = angle * (180.0 / 3.141592653589793238463);
    std::cout << "the angle between two planes is " << angle_deg << std::endl;
    std::cout << "the error is: " << abs((angle_deg - 90) / 90.0) * 100.0 << "%" << std::endl;

    delete viewer;
    return planes;
}

std::vector<float> segmentation::get_plane_coefs(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud) {
    pcl::PointIndices inliers;
    pcl::ModelCoefficients coefficients;

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    coefficients.values.resize(4);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.005);
    seg.setInputCloud(cloud);
    seg.segment(inliers, coefficients);

    std::cout << "Model coefficients: " << coefficients.values[0] << " "
              << coefficients.values[1] << " "
              << coefficients.values[2] << " "
              << coefficients.values[3] << std::endl;

    return coefficients.values;
}