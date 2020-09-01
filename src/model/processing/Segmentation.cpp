#include "Segmentation.h"
#include "Visualizer.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>


std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> segmentation::remove_plane(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloudIn) {
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloudInliers(new pcl::PointCloud<pcl::PointXYZ>);
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    coefficients->values.resize(4);
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.005);

    seg.setInputCloud(cloudIn);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty()) {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    }

    std::cerr << "Model coefficients: " << coefficients->values[0] << " "
              << coefficients->values[1] << " "
              << coefficients->values[2] << " "
              << coefficients->values[3] << std::endl;


    // Extract inliers
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloudIn);
    extract.setIndices(inliers);
    extract.setNegative(false);            // Extract the inliers
    extract.filter(*cloudInliers);        // cloud_inliers contains the plane

    // Extract outliers
    extract.setNegative(true);                // Extract the outliers
    extract.filter(*cloudOutliers);        // cloud_outliers contains everything but the plane
    return cloudOutliers;
}

std::vector<equations::Plane> segmentation::get_calibration_planes_coefs(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud) {
    std::vector<equations::Plane> planes;
    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> temp_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_f(new pcl::PointCloud<pcl::PointXYZ>());
    *temp_cloud = *cloud;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.002);

    int i = 0, nr_points = (int) temp_cloud->points.size();
    while (temp_cloud->points.size() > 0.3 * nr_points) {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(temp_cloud);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0) {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(temp_cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);

        // Get the points associated with the planar surface
        extract.filter(*cloud_plane);
        planes.emplace_back(coefficients);

        std::cout << "Model coefficients: " << coefficients->values[0] << " "
                  << coefficients->values[1] << " "
                  << coefficients->values[2] << " "
                  << coefficients->values[3] << std::endl;
        visual::Visualizer visualizer;
        std::vector<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> clouds{temp_cloud, cloud_plane};
        visualizer.simpleVis(clouds);

        // Remove the planar inliers, extract the rest
        extract.setNegative(true);
        extract.filter(*cloud_f);
        *temp_cloud = *cloud_f;
    }
    return planes;
}

std::vector<float> segmentation::get_plane_coefs(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud) {
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    coefficients->values.resize(4);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.005);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    std::cout << "Model coefficients: " << coefficients->values[0] << " "
              << coefficients->values[1] << " "
              << coefficients->values[2] << " "
              << coefficients->values[3] << std::endl;

    return coefficients->values;
}