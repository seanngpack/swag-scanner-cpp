#ifndef SWAG_SCANNER_DEPTH_H
#define SWAG_SCANNER_DEPTH_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace camera {
    class intrinsics;
}

/**
 * Contains implementation for converting depth frames to point clouds.
 */
namespace depth {
    /**
     * Create a PointCloudXYZ given a depth frame and camera intrin.
     * @param depth_frame vector of uint16_t depth values. Not converted to meters yet.
     * @return a pointcloud.
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr create_point_cloud(const std::vector<uint16_t> &depth_frame,
                                                           const camera::intrinsics intrinsics);
}

#endif //SWAG_SCANNER_DEPTH_H
