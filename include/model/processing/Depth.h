/**
 * Contains implementatoin for converting depth frames to point clouds.
 */

#ifndef SWAG_SCANNER_DEPTH_H
#define SWAG_SCANNER_DEPTH_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "CameraTypes.h"
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

namespace depth {
    /**
     * Create a PointCloudXYZ given a depth frame.
     * @param depth_frame a pointer to vector of uint16_t. Not converted to meters yet.
     * @return a pointcloud.
     */
    inline pcl::PointCloud<pcl::PointXYZ>::Ptr create_point_cloud(const uint16_t *depth_frame,
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

#endif //SWAG_SCANNER_DEPTH_H
