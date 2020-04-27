#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#ifndef SWAG_SCANNER_ALGORITHMS_H
#define SWAG_SCANNER_ALGORITHMS_H

namespace algos {


    /**
     * Given three points, deproject their pixel coordinates to space coordinates and
     * then save to a PointXYZ format.
     * @param x pixel x.
     * @param y pixel y.
     * @param z depth (unconverted).
     * @return a PointXYZ object with the deprojected point in real space.
     */
    pcl::PointXYZ deproject_pixel_to_point(int x,
                                           int y,
                                           uint16_t z,
                                           const camera::ss_intrinsics *intrinsics) {
        float depth = z * intrinsics->depth_scale;
        float x_d = x - intrinsics->ppx * depth * (1 / intrinsics->fx);
        float y_d = y - intrinsics->ppy * depth * (1 / intrinsics->fy);
        pcl::PointXYZ point = pcl::PointXYZ(x_d, y_d, depth);
        return point;
    }

    /**
     * Create a PointCloudXYZ given a depth frame.
     * @param depth_frame a pointer to vector of uint16_t. Not converted to meters yet.
     * @return a pointcloud.
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr create_point_cloud(const uint16_t *depth_frame,
                                                           const camera::ss_intrinsics *intrinsics) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud->height = intrinsics->height;
        cloud->width = intrinsics->width;

        for (int y = 0; y < intrinsics->height; y++) {
            for (int x = 0; x < intrinsics->width; x++) {
                pcl::PointXYZ point;
                uint16_t depth = depth_frame[y * intrinsics->width + x];
//                if (depth == 0) continue;
                point = deproject_pixel_to_point(x, y, depth, intrinsics);
                cloud->push_back(point);
            }
        }
        return cloud;
    }


}

#endif //SWAG_SCANNER_ALGORITHMS_H
