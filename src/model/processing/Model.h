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
        * Create a new PointCloudXYZ using the instance variable depth_frame.
        * @return a boost pointer to the new pointcloud.
        */
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> create_point_cloud(const std::vector<uint16_t> &depth_frame,
                                                                           const camera::intrinsics &intrinsics);

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
         * Applies crop box filtering to remove outside points from cloud.
         * @param cloud the cloud you want to crop.
         * @param croppedCloud the cropped cloud.
         * @return the cropped cloud.
         */
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> crop_cloud(
                const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud,
                float minX, float maxX,
                float minY, float maxY,
                float minZ, float maxZ);

        /**
         * Downsample the given cloud using voxel grid.
         * @param cloud cloud you want to downsample.
         * @param leafSize size of leaf.
         * @return the downsampled cloud.
         */
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>
        voxel_grid_filter(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud,
                          float leafSize = .01);

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
         * Get the upright and ground plane equations.
         *
         * @param cloud calibration cloud.
         * @param visual_flag flag whether to visualize segmentation or not.
         * @return vector of upright and ground plane equations.
         */
        std::vector<equations::Plane>
        get_calibration_planes_coefs(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud,
                                     bool visual_flag = false);

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
         * Given a vector of ground planes, calculate the axis direction by taking the
         * average of the coefficients.
         * @param ground_planes vector of ground plane equations.
         * @return the axis direction.
         */
        equations::Normal calculate_axis_dir(const std::vector<equations::Plane> &ground_planes);

        /**
         * Given a vector of planes, average them.
         *
         * @param planes planes.
         * @return average of the planes.
         */
        equations::Plane average_planes(const std::vector<equations::Plane> &planes);

        /**
         * Calculate the origin point of the turntable using equation of rotation axis and equations
         * for the upright planes.
         * @param axis_dir direction of rotation axis.
         * @param upright_planes vector of upright planes.
         * @return the origin point.
         */
        pcl::PointXYZ calculate_center_pt(const equations::Normal &axis_dir,
                                          const std::vector<equations::Plane> &upright_planes);

        /**
         * Project center point to ground plane.
         *
         * @param cloud the cloud that the plane you are projecting to belongs to.
         * @param pt point you want to project.
         * @param plane the plane you want to project onto.
         * @param delta the threshold to search for point on plane.
         * @return projected point on the plane.
         */
        pcl::PointXYZ refine_center_pt(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud,
                                       const pcl::PointXYZ &pt,
                                       const equations::Plane &plane,
                                       double delta = .00001
        );


        /**
         * Find a point lying on the given plane in the cloud.
         *
         * @param cloud cloud.
         * @param plane plane.
         * @param delta error threshold for finding the point.
         * @return point in the plane or point of 0,0,0.
         */
        pcl::PointXYZ find_point_in_plane(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud,
                                          const equations::Plane &plane,
                                          double delta = .1);


        /**
         * Rotate a point cloud about a line.
         *
         * @param cloud the cloud you want to rotate. Must be an unorganized cloud.
         * @param pt a point on the axis you want to rotate about.
         * @param line_direction direction vector for the line (normalized)
         * @param theta angle in radians you want to rotate.
         * @return the rotated cloud.
         */
        pcl::PointCloud<pcl::PointXYZ>
        rotate_cloud_about_line(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud,
                                const std::vector<float> &pt,
                                const std::vector<float> &line_direction,
                                float theta);

        /**
         * Rotate cloud about the z-axis.
         *
         * @param cloud cloud to rotate.
         * @param theta rotation degree.
         * @return
         */
        pcl::PointCloud<pcl::PointXYZ>
        rotate_cloud_about_z_axis(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud,
                                  float theta);

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
