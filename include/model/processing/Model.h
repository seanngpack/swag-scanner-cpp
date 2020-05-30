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
         * Using integral images to compute normals much faster than standard plane fitting.
         * @return a normal cloud.
         */
        pcl::PointCloud<pcl::Normal>::Ptr estimate_normal_cloud(
                pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);


        /**
         * Save pointcloud to file.
         * @param cloud the cloud you want to save.
         */
        void to_file(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                     const std::string &name,
                     CloudType::Type cloud_type);


        ~Model();

    private:
        file::FileHandler fileHandler;
    };
}

#endif //SWAG_SCANNER_MODEL_H
