#include "Registration.h"
#include <pcl/registration/icp.h>
#include <pcl/registration/ia_ransac.h>

Eigen::Matrix4f registration::icp_register_pair_clouds(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloudIn,
                                                       const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloudOut,
                                                       const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &transformedCloud) {
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloudIn);
    icp.setInputTarget(cloudOut);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-1);
    icp.setMaxCorrespondenceDistance (.05); // not really sure how this affects results
    icp.setEuclideanFitnessEpsilon(.0001); // big effect
    icp.setRANSACOutlierRejectionThreshold(.0001); // doesn't seem to affect results much
    std::cout << "registering clouds..." << std::endl;
    icp.align(*transformedCloud);
    if (icp.hasConverged()) {
        std::cout << "\nICP has converged, score is " << icp.getFitnessScore() << std::endl;
        auto trans = icp.getFinalTransformation().cast<double>();
        std::cout << trans << std::endl;
    } else {
        PCL_ERROR ("\nICP has not converged.\n");
    }
    return icp.getFinalTransformation();
}

void registration::sac_align_pair_clouds(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloudIn,
                                         const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloudTarget,
                                         const std::shared_ptr<pcl::PointCloud<pcl::FPFHSignature33>> &cloudInFeatures,
                                         const std::shared_ptr<pcl::PointCloud<pcl::FPFHSignature33>> &cloudOutFeatures,
                                         const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloudAligned,
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