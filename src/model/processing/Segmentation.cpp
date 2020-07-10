#include "Segmentation.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>


pcl::PointCloud<pcl::PointXYZ>::Ptr segmentation::remove_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudIn) {
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>);
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

std::vector<equations::Plane> segmentation::get_calibration_planes_coefs(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    std::vector<equations::Plane> planes;
    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>());
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.002);

    int i = 0, nr_points = (int) cloud->points.size();
    while (cloud->points.size() > 0.3 * nr_points) {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0) {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);

        // Get the points associated with the planar surface
        extract.filter(*cloud_plane);
        planes.emplace_back(coefficients);


//            std::cout << "Model coefficients: " << coefficients->values[0] << " "
//                      << coefficients->values[1] << " "
//                      << coefficients->values[2] << " "
//                      << coefficients->values[3] << std::endl;
//            visual::Visualizer visualizer;
//            std::vector<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> clouds{cloud, cloud_plane};
//            visualizer.simpleVis(clouds);

        // Remove the planar inliers, extract the rest
        extract.setNegative(true);
        extract.filter(*cloud_f);
        *cloud = *cloud_f;
    }
    return planes;
}

std::vector<float> segmentation::get_plane_coefs(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
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