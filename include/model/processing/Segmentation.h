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


    inline pcl::PointCloud<pcl::PointXYZ>::Ptr remove_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudIn) {
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

    /**
     * Given a calibration cloud, extract the ground and upright plane.
     * @param cloud calibration cloud.
     * @return a vector of coefficients for the two planes.
     */
    inline std::vector<pcl::ModelCoefficients::Ptr> get_calibration_planes_coefs(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
        std::vector<pcl::ModelCoefficients::Ptr> coefs_vector;
// Create the segmentation object for the planar model and set all the parameters
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ> ());
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (100);
        seg.setDistanceThreshold (0.002);

        int i=0, nr_points = (int) cloud->points.size ();
        while (cloud->points.size () > 0.3 * nr_points)
        {
            // Segment the largest planar component from the remaining cloud
            seg.setInputCloud (cloud);
            seg.segment (*inliers, *coefficients);
            if (inliers->indices.size () == 0)
            {
                std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
                break;
            }

            // Extract the planar inliers from the input cloud
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud (cloud);
            extract.setIndices (inliers);
            extract.setNegative (false);

            // Get the points associated with the planar surface
            extract.filter (*cloud_plane);

            std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

            std::cout << "Model coefficients: " << coefficients->values[0] << " "
                      << coefficients->values[1] << " "
                      << coefficients->values[2] << " "
                      << coefficients->values[3] << std::endl;

            coefs_vector.push_back(coefficients);

            ///
            visual::Visualizer visualizer;
            std::vector<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> clouds{cloud, cloud_plane};
            visualizer.simpleVis(clouds);

            ///

            // Remove the planar inliers, extract the rest
            extract.setNegative (true);
            extract.filter (*cloud_f);
            *cloud = *cloud_f;
        }
        return coefs_vector;
    }

    /**
     * Get the plane coefficients of the base of the given cloud.
     * @param cloud input cloud.
     * @return coefficients vector of size 4 of the cloud.
     */
    inline std::vector<float> get_plane_coefs(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
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
}

#endif //SWAG_SCANNER_SEGMENTATION_H
