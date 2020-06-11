/**
 * Model for processing point clouds. Holds a reference to the FileHandler for saving.
 */
#ifndef SWAG_SCANNER_MODEL_H
#define SWAG_SCANNER_MODEL_H

#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ia_ransac.h>
#include <CameraTypes.h>
#include <CloudType.h>
#include "Visualizer.h"

namespace model {

    /**
     * Procesing model that contains functions to manipulate depth frames and pointclouds.
     */
    class Model {

    public:
        /**
         * Constructor for Model.
         */
        Model();

        /**
        * Create a new PointCloudXYZ using the instance variable depth_frame.
        * @return a boost pointer to the new pointcloud.
        */
        pcl::PointCloud<pcl::PointXYZ>::Ptr create_point_cloud(const uint16_t *depth_frame,
                                                               const camera::ss_intrinsics *intrinsics);

        /**
         * Take in a pointcloud, calculate the normals, and return a normal cloud.
         * @return a normal cloud.
         */
        pcl::PointCloud<pcl::Normal>::Ptr estimate_normal_cloud(
                pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);

        /**
         * Given a cloud and its normal, calculate the features.
         * @param cloud the cloud you want to find features for.
         * @param normalCloud normlas of cloud.
         * @param features features.
         */
        void compute_local_features(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                    pcl::PointCloud<pcl::Normal>::Ptr normalCloud,
                                    pcl::PointCloud<pcl::FPFHSignature33>::Ptr features);

        /**
         * Applies crop box filtering to remove outside points from cloud.
         * @param cloud the cloud you want to crop.
         * @param croppedCloud the cropped cloud.
         * @return the cropped cloud.
         */
        pcl::PointCloud<pcl::PointXYZ>::Ptr crop_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                                       float minX, float maxX,
                                                       float minY, float maxY,
                                                       float minZ, float maxZ);

        /**
         * Downsample the given cloud using voxel grid.
         * @param cloud cloud you want to downsample.
         * @param leafSize size of leaf.
         * @return the downsampled cloud.
         */
        pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_grid_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                                              float leafSize = .01);

        /**
         * Remove scanning bed plane from the cloud.
         * @param cloudIn cloud you want to remove the plane from.
         * @return cloud with the remove plane
         */
        pcl::PointCloud<pcl::PointXYZ>::Ptr remove_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudIn);

        /**
         * Use ICP to register an input and target cloud.
         * @returns a transformation matrix from the source to target cloud.
         */
        Eigen::Matrix4f register_pair_clouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn,
                                             pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut,
                                             pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud);

        /**
         * Use SAC to get initial cloud alignment.
         * @param cloudIn input cloud for template.
         * @param cloudTarget align to template.
         * @param cloudAligned output, should be the input cloud aligned to template.
         */
        void align_clouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTarget,
                          pcl::PointCloud<pcl::FPFHSignature33>::Ptr cloudInFeatures,
                          pcl::PointCloud<pcl::FPFHSignature33>::Ptr cloudOutFeatures,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr cloudAligned,
                          Eigen::Matrix4f &transformation);


        /**
         * Overloaded method that will calculate the features given just the in and target clouds.
         * @param cloudIn
         * @param cloudTarget
         * @param cloudAligned
         */
        void align_clouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTarget,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr cloudAligned,
                          Eigen::Matrix4f &transformation);


        ~Model();

    private:

    };
}

#endif //SWAG_SCANNER_MODEL_H
