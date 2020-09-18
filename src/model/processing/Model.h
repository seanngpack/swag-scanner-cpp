/**
 * Model for processing point clouds. Holds a reference to the IFileHandler for saving.
 */
#ifndef SWAG_SCANNER_MODEL_H
#define SWAG_SCANNER_MODEL_H

#include "CloudType.h"
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <memory>

namespace camera {
    class intrinsics;
}

namespace equations {
    class Normal;

    class Point;

    class Plane;
}

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
         * Take in a pointcloud, calculate the normals, and return a normal cloud.
         * @return a normal cloud.
         */
        std::shared_ptr<pcl::PointCloud<pcl::Normal>>
        estimate_normal_cloud(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &point_cloud);

        /**
         * Given a cloud and its normal, calculate the features.
         * @param cloud the cloud you want to find features for.
         * @param normal_cloud normals of cloud.
         * @return the features of the given normal cloud.
         */
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr compute_local_features(
                const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud,
                const std::shared_ptr<pcl::PointCloud<pcl::Normal>> &normal_cloud);





        /**
         * Remove outliers from cloud.
         *
         * @param cloud cloud to filter.
         * @param mean_k number of neighbors to analyze.
         * @param thresh_mult multipler for standard deviation, members outside st will be removed.
         * @return filtered cloud.
         */
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>
        remove_outliers(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud,
                        float mean_k = 50,
                        float thresh_mult = 1);

        /**
         * Remove NaN points from cloud.
         *
         * @param cloud cloud to remove points from.
         * @return cloud without NaN points.
         */
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>
        remove_nan(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud);



        /**
         * Get the coefficients of the base plane of the given cloud.
         * @param cloud input cloud.
         * @return vector of size 4 of the plane coefficients.
         */
        std::vector<float> get_plane_coefs(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud);

        /**
         * Remove scanning bed plane from the cloud.
         * @param cloudIn cloud you want to remove the plane from.
         * @return cloud with the remove plane
         */
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>
        remove_plane(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloudIn);






        /**
         * Transform (translate and rotate) given cloud to center it at world origin coordinate (0,0,0)
         * Z axis pointer up.
         *
         * @param cloud cloud to transform.
         * @param center the center coordinate of turntable.
         * @param rotation_axis direction vector of ground.
         * @return
         */
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>
        transform_cloud_to_world(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud,
                                 const pcl::PointXYZ &center,
                                 const equations::Normal &rotation_axis);

        /**
         * Use ICP to register an input and target cloud.
         * @param cloud_in input cloud.
         * @param cloud_target target cloud.
         * @param transformed_cloud the final transformed cloud.
         * @returns a transformation matrix from the source to target cloud.
         */
        Eigen::Matrix4f icp_register_pair_clouds(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud_in,
                                                 const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud_target,
                                                 std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &transformed_cloud);


        /**
         * Find initial alignment of two clouds using FPFH.
         * @param cloudIn pointcloud.
         * @param cloudTarget target cloud.
         * @param cloudAligned the aligned cloud.
         */
        void sac_align_pair_clouds(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloudIn,
                                   const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloudTarget,
                                   const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloudAligned,
                                   Eigen::Matrix4f &transformation);


        ~Model() = default;

    private:

    };
}

#endif //SWAG_SCANNER_MODEL_H
