#include "Registration.h"
#include <pcl/registration/icp.h>
#include <pcl/registration/ia_ransac.h>

Eigen::Matrix4f registration::icp_register_pair_clouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn,
                                                pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut,
                                                pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud) {
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloudIn);
    icp.setInputTarget(cloudOut);
//    icp.setMaximumIterations (200);
//    icp.setTransformationEpsilon(1e-9);
    icp.setMaxCorrespondenceDistance(0.002); // 2mm
//    icp.setEuclideanFitnessEpsilon (1);
//    icp.setRANSACOutlierRejectionThreshold (1.5);
    std::cout << "registering clouds..." << std::endl;
    icp.align(*transformedCloud);
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
              icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;
    return icp.getFinalTransformation();
}

void registration::sac_align_pair_clouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn,
                                  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTarget,
                                  pcl::PointCloud<pcl::FPFHSignature33>::Ptr cloudInFeatures,
                                  pcl::PointCloud<pcl::FPFHSignature33>::Ptr cloudOutFeatures,
                                  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudAligned,
                                  Eigen::Matrix4f &transformation) {
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> reg;
    reg.setMinSampleDistance(0.05f);
    reg.setMaxCorrespondenceDistance(0.1);
    reg.setMaximumIterations(1000);

    reg.setInputSource(cloudIn);
    reg.setInputTarget(cloudTarget);
    reg.setSourceFeatures(cloudInFeatures);
    reg.setTargetFeatures(cloudOutFeatures);
    reg.align(*cloudAligned);
    transformation = reg.getFinalTransformation();
}