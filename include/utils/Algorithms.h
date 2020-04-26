#include <Eigen/Dense>

#ifndef SWAG_SCANNER_ALGORITHMS_H
#define SWAG_SCANNER_ALGORITHMS_H

namespace algos {

    /**
     * Deproject depth frame to a R X C dynamic matrix.
     * @param depth_frame a pointer to a vector representing the depth frame obtained from camera.
     * @return a dynamic matrix with the deprojectoed points.
     */
    Eigen::MatrixXf deproject_depth_frame(const uint16_t *depth_frame) {

    }


    template <typename Derived>
    /**
     * Convert a depth matrix to a pointcloud.
     * @tparam Derived generic eigen object.
     * @param depth_matrix (row x col) matrix containing values in meters of depth mapping.
     * @return a boost smart pointer to the formed pointcloud.
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr matrix_to_point_cloud(const Eigen::MatrixBase<Derived>& depth_matrix) {

    }


}

#endif //SWAG_SCANNER_ALGORITHMS_H
