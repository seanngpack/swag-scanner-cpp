#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/integral_image_normal.h>
#include <FileHandler.h>
#include <CameraTypes.h>
#include <CloudType.h>

#ifndef SWAG_SCANNER_MODEL_H
#define SWAG_SCANNER_MODEL_H

namespace model {

    /**
     * Procesing model that contains functions to manipulate depth frames and pointclouds.
     */
    class Model {

    public:
        Model();

        /**
         * Set the depth_frame instance variable.
         * @param depth_frame the depth frame you want to set.
         */
        void set_depth_frame(const uint16_t *depth_frame);

        /**
         * Give the point cloud for the class to hold onto.
         * @param cloud the cloud you want to set.
         */
        void set_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

        /**
         * Set the intrinsics to a pointer to its memory address.
         * @param intrinsics the camera intrinsics.
         */
        void set_intrinsics(const camera::ss_intrinsics *intrinsics);

        /**
         * Get the depth frame.
         * @return the depth frame.
         * @throws Runtime error if a depth frame is not set yet.
         */
        const uint16_t *get_depth_frame();

        /**
         * Return the pointer to point cloud.
         * @returns pointcloud.
         */
        pcl::PointCloud<pcl::PointXYZ>::Ptr get_point_cloud();

        /**
         * Return the pointer to a normal cloud.
         * @return normal cloud.
         */
        pcl::PointCloud<pcl::Normal>::Ptr get_normal_cloud();

        /**
         * Get the intrinsics.
         * @return the intrinsics.
         * @throws runtime error is the intrinsics is not set yet.
         */
        const camera::ss_intrinsics *get_intrinsics();

        /**
        * Create a new PointCloudXYZ using the instance variable depth_frame.
        * @return a boost pointer to the new pointcloud.
        */
        pcl::PointCloud<pcl::PointXYZ>::Ptr create_point_cloud();

        /**
         * Take in a pointcloud, calculate the normals, and return a normal cloud.
         * Using integral images to compute normals much faster than standard plane fitting.
         * @return a normal cloud.
         */
        pcl::PointCloud<pcl::Normal>::Ptr estimate_normal_cloud();

        /**
         * Save pointcloud to file.
         * @param cloud the cloud you want to save.
         */
        void to_file(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                     const std::string &name,
                     CloudType::Type cloud_type);


        ~Model();

    private:
        const uint16_t *depth_frame;
        const camera::ss_intrinsics *intrinsics;
        file::FileHandler fileHandler;
        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud;
        pcl::PointCloud<pcl::Normal>::Ptr normal_cloud;

    };
}

#endif //SWAG_SCANNER_MODEL_H
