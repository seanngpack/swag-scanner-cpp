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