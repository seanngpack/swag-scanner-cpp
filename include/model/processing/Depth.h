#ifndef SWAG_SCANNER_DEPTH_H
#define SWAG_SCANNER_DEPTH_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "CameraTypes.h"

/**
 * Contains implementation for converting depth frames to point clouds.
 */
namespace depth {
    /**
     * Create a PointCloudXYZ given a depth frame and camera intrinsics.
     * @param depth_frame a pointer to vector of uint16_t. Not converted to meters yet.
     * @return a pointcloud.
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr create_point_cloud(const uint16_t *depth_frame,
                                                           const camera::ss_intrinsics *intrinsics);
}

#endif //SWAG_SCANNER_DEPTH_H
