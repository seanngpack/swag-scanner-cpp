#ifndef SWAG_SCANNER_SEGMENTATION_H
#define SWAG_SCANNER_SEGMENTATION_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
//#include <Eigen/Dense>

namespace segmentation {


    inline void remove_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudIn,
                             pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudOut) {
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
        cloudOut = cloudOutliers;
        std::cout << cloudOut->size() << std::endl;
        std::cout << cloudOut->points[100] << std::endl;
    }

}

#endif //SWAG_SCANNER_SEGMENTATION_H
