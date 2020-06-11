#include "Model.h"
#include "Depth.h"
#include "Filtering.h"
#include "Segmentation.h"

model::Model::Model(){}


pcl::PointCloud<pcl::PointXYZ>::Ptr model::Model::create_point_cloud(const uint16_t *depth_frame,
                                                                     const camera::ss_intrinsics *intrinsics) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud = depth::create_point_cloud(depth_frame, intrinsics);
    return point_cloud;
}

pcl::PointCloud<pcl::Normal>::Ptr model::Model::estimate_normal_cloud(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.03);

    ne.setInputCloud(cloud);
    std::cout << "estimating normal cloud" << std::endl;
    std::cout << cloud->size() << std::endl;
    ne.compute(*normals);
    return normals;
}

void model::Model::compute_local_features(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                          pcl::PointCloud<pcl::Normal>::Ptr normalCloud,
                                          pcl::PointCloud<pcl::FPFHSignature33>::Ptr features) {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr searchMethod(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
    fpfh_est.setInputCloud(cloud);
    fpfh_est.setInputNormals(normalCloud);
    fpfh_est.setSearchMethod(searchMethod);
    fpfh_est.setRadiusSearch(.05);
    fpfh_est.compute(*features);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr model::Model::crop_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                                             float minX, float maxX,
                                                             float minY, float maxY,
                                                             float minZ, float maxZ) {
    return filtering::crop_cloud(cloud, minX, maxX, minY, maxY, minZ, maxZ);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr model::Model::voxel_grid_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                                                    float leafSize) {
    return filtering::voxel_grid_filter(cloud, leafSize);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr model::Model::remove_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudIn) {
    return segmentation::remove_plane(cloudIn);
}

Eigen::Matrix4f model::Model::register_pair_clouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn,
                                                   pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut,
                                                   pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud) {
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloudIn);
    icp.setInputTarget(cloudOut);
//    icp.setMaximumIterations (200);
    icp.setTransformationEpsilon(1e-9);
    icp.setMaxCorrespondenceDistance(0.01);
//    icp.setEuclideanFitnessEpsilon (1);
//    icp.setRANSACOutlierRejectionThreshold (1.5);
    std::cout << "registering clouds..." << std::endl;
    icp.align(*transformedCloud);
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
              icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;
    return icp.getFinalTransformation();

}

void model::Model::align_clouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn,
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

void model::Model::align_clouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTarget,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr cloudAligned,
                                Eigen::Matrix4f &transformation) {
    pcl::PointCloud<pcl::Normal>::Ptr cloudInNormal = estimate_normal_cloud(cloudIn);
    pcl::PointCloud<pcl::Normal>::Ptr cloudTargetNormal = estimate_normal_cloud(cloudTarget);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr cloudInFeatures(new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr cloudTargetFeatures(new pcl::PointCloud<pcl::FPFHSignature33>);

    std::cout << "computing local features of cloud" << std::endl;
    compute_local_features(cloudIn, cloudInNormal, cloudInFeatures);
    compute_local_features(cloudTarget, cloudTargetNormal, cloudTargetFeatures);


    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> reg;
    reg.setMinSampleDistance(0.05f);
    reg.setMaxCorrespondenceDistance(0.1);
    reg.setMaximumIterations(1000);

    reg.setInputSource(cloudIn);
    reg.setInputTarget(cloudTarget);
    reg.setSourceFeatures(cloudInFeatures);
    reg.setTargetFeatures(cloudTargetFeatures);
    std::cout << "aligning..." << std::endl;
    reg.align(*cloudAligned);
    transformation = reg.getFinalTransformation();
}

model::Model::~Model() {
    std::cout << "calling model destructor \n";;
}








