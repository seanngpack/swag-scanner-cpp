#include "Registration.h"
#include <pcl/registration/icp.h>
#include <pcl/registration/ia_ransac.h>

Eigen::Matrix4f registration::icp_register_pair_clouds(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloudIn,
                                                std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloudOut,
                                                std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> transformedCloud) {
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

void registration::sac_align_pair_clouds(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloudIn,
                                  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloudTarget,
                                  pcl::PointCloud<pcl::FPFHSignature33>::Ptr cloudInFeatures,
                                  pcl::PointCloud<pcl::FPFHSignature33>::Ptr cloudOutFeatures,
                                  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloudAligned,
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