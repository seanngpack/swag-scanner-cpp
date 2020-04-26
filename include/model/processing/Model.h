#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <CameraTypes.h>

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
         * Create a new PointCloudXYZ using the instance variable depth_frame.
         * @return a boost pointer to the new pointcloud.
         */
        pcl::PointCloud<pcl::PointXYZ>::Ptr create_point_cloud();

        /**
         * Set the depth_frame instance variable.
         * @param depth_frame the depth frame you want to set.
         */
        void set_depth_frame(const uint16_t *depth_frame);

        ~Model();

    private:
        const uint16_t *depth_frame;
        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud;


    };
}

#endif //SWAG_SCANNER_MODEL_H
