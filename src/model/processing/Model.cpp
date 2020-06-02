#include "Model.h"
#include "Depth.h"
#include "Filtering.h"

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

void model::Model::to_file(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                           const std::string &name,
                           CloudType::Type cloud_type) {
    if (!auto_create_folders) {
        // this error should be changed later, read the todo in filehandler.h
        throw std::runtime_error("Cannot save file, filehandler save path not set");
    }
    fileHandler.save_cloud(cloud, name, cloud_type);

}

model::Model::~Model() {
    std::cout << "calling model destructor \n";;
}



