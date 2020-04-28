#ifndef SWAG_SCANNER_ALGORITHMS_H
#define SWAG_SCANNER_ALGORITHMS_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "CameraTypes.h"
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

namespace algos {

    /**
     * Given three points, deproject their pixel coordinates to space coordinates and
     * then save to a PointXYZ format.
     * @param x pixel x.
     * @param y pixel y.
     * @param z depth (unconverted).
     * @return a PointXYZ object with the deprojected point in real space.
     */
    pcl::PointXYZ deproject_pixel_to_point(float x_pixel,
                                           float y_pixel,
                                           float z,
                                           const camera::ss_intrinsics *intrinsics) {
        float depth = z * intrinsics->depth_scale;
        float x = (x_pixel - intrinsics->ppx) / intrinsics->fx;
        float y = (y_pixel - intrinsics->ppy) / intrinsics->fy;
        float ux = x * depth;
        float uy = y * depth;

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

                uint16_t depth = depth_frame[y * intrinsics->width + x];
                float depth_in_meters = depth * intrinsics->depth_scale;
                if (depth == 0) continue;
//                point = deproject_pixel_to_point((float)x, (float)y, (float)depth, intrinsics);
                float pixel[2] = {(float) x, (float) y};
                float point_array[3] = {(float) x, (float) y, 0};
                const rs2_intrinsics intrin = {intrinsics->width, intrinsics->height,
                                               intrinsics->ppx, intrinsics->ppy,
                                               intrinsics->fx, intrinsics->fy,
                                               intrinsics->model, *intrinsics->coeffs};
                const rs2_intrinsics *ptr = &intrin;
                rs2_deproject_pixel_to_point(point_array, ptr, pixel, depth_in_meters);
                pcl::PointXYZ point;
                point.x = point_array[0];
                point.y = point_array[1];
                point.z = point_array[2];
                cloud->points[y * intrinsics->width + x] = point;
            }
        }

        return cloud;
    }
}

#endif //SWAG_SCANNER_ALGORITHMS_H
