#include "Model.h"
#include "Depth.h"
#include "Filtering.h"
#include "Segmentation.h"

model::Model::Model(bool auto_create_folders) :
        auto_create_folders(auto_create_folders),
        fileHandler(auto_create_folders) {}


pcl::PointCloud<pcl::PointXYZ>::Ptr model::Model::create_point_cloud(const uint16_t *depth_frame,
                                                                     const camera::ss_intrinsics *intrinsics) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud = depth::create_point_cloud(depth_frame, intrinsics);
    return point_cloud;
}

pcl::PointCloud<pcl::Normal>::Ptr model::Model::estimate_normal_cloud(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(cloud);
    ne.compute(*normals);
    return normals;
}

void model::Model::crop_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                              float minX, float maxX,
                              float minY, float maxY,
                              float minZ, float maxZ) {
    filtering::crop_cloud(cloud, minX, maxX, minY, maxY, minZ, maxZ);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr model::Model::voxel_grid_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                                                    float leafSize) {
    return filtering::voxel_grid_filter(cloud, leafSize);
}

void model::Model::remove_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut) {
    segmentation::remove_plane(cloudIn, cloudOut);
}

Eigen::Matrix4f model::Model::register_pair_clouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn,
                                                   pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut,
                                                   pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud) {
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloudIn);
    icp.setInputTarget(cloudOut);
//    icp.setMaximumIterations (50);
//    icp.setTransformationEpsilon (1e-9);
//    icp.setMaxCorrespondenceDistance (0.05);
//    icp.setEuclideanFitnessEpsilon (1);
//    icp.setRANSACOutlierRejectionThreshold (1.5);
//    pcl::PointCloud<pcl::PointXYZ> Final;
    std::cout << "registering clouds..." << std::endl;
    icp.align(*transformedCloud);
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
              icp.getFitnessScore() << std::endl;
    return icp.getFinalTransformation();

}


void model::Model::save_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                              const std::string &name,
                              CloudType::Type cloud_type) {
    if (!auto_create_folders) {
        // this error should be changed later, read the todo in filehandler.h
        throw std::runtime_error("Cannot save file, filehandler save path not set");
    }
    fileHandler.save_cloud(cloud, name, cloud_type);

}

void model::Model::load_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::string &cloud_name,
                              CloudType::Type cloud_type) {
    fileHandler.load_cloud(cloud, cloud_name, cloud_type);
}

void model::Model::load_clouds(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr,
        Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ>::Ptr>> &cloud_vector,
                               CloudType::Type cloud_type,
                               const std::string &folder_path) {
    fileHandler.load_clouds(cloud_vector, cloud_type, folder_path);
}

model::Model::~Model() {
    std::cout << "calling model destructor \n";;
}








