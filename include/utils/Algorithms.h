#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "CameraTypes.h"

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
        float ux = (x - intrinsics->ppx) * (1 / intrinsics->fx) * depth;
        float uy = (y - intrinsics->ppy) * (1 / intrinsics->fy) * depth;
        pcl::PointXYZ point = pcl::PointXYZ(ux, uy, depth);
        return point;
    }

    /**
     * Create a PointCloudXYZ given a depth frame.
     * @param depth_frame a pointer to vector of uint16_t. Not converted to meters yet.
     * @return a pointcloud.
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr create_point_cloud(const uint16_t *depth_frame,
                                                           const camera::ss_intrinsics *intrinsics) {
        // cloud setup
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud->height = intrinsics->height;
        cloud->width = intrinsics->width;
        cloud->is_dense = true;
        cloud->points.resize(intrinsics->width * intrinsics->height);
        for (int y = 0; y < intrinsics->height; y++) {
            for (int x = 0; x < intrinsics->width; x++) {
                pcl::PointXYZ point;
                uint16_t depth = depth_frame[y * intrinsics->width + x];
//                if (depth == 0) continue; // pretty sure this makes the cloud not dense.
                point = deproject_pixel_to_point(x, y, depth, intrinsics);
                cloud->points[y * intrinsics->width + x] = point;
            }
        }

        return cloud;
    }
}

#endif //SWAG_SCANNER_ALGORITHMS_H
