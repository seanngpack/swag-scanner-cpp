#ifndef SWAG_SCANNER_REGISTRATION_H
#define SWAG_SCANNER_REGISTRATION_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace registration {

    /**
     * Use ICP to register a pair of pointclouds.
     * @param cloudIn source cloud..
     * @param cloudOut target cloud.
     * @param transformedCloud empty cloud used as output.
     * @return transformation matrix source -> target.
     */
    Eigen::Matrix4f icp_register_pair_clouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn,
                                             pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut,
                                             pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud);

    /**
     * Use SAC model to align cloud to target.
     * @param cloudIn source cloud.
     * @param cloudTarget target cloud.
     * @param cloudInFeatures features of source cloud.
     * @param cloudOutFeatures features of target cloud.
     * @param cloudAligned output cloud source -> target.
     * @param transformation output transformation.
     */
    inline void sac_align_pair_clouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn,
                                      pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTarget,
                                      pcl::PointCloud<pcl::FPFHSignature33>::Ptr cloudInFeatures,
                                      pcl::PointCloud<pcl::FPFHSignature33>::Ptr cloudOutFeatures,
                                      pcl::PointCloud<pcl::PointXYZ>::Ptr cloudAligned,
                                      Eigen::Matrix4f &transformation);
}

#endif
