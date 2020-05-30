#include "Model.h"
#include "Depth.h"

model::Model::Model()
        : depth_frame(nullptr),
          intrinsics(nullptr),
          fileHandler(true),
          point_cloud(nullptr),
          normal_cloud(nullptr) {}


void model::Model::set_depth_frame(const uint16_t *depth_frame) {
    this->depth_frame = depth_frame;
}


void model::Model::set_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    this->point_cloud = cloud;
}

void model::Model::set_intrinsics(const camera::ss_intrinsics *intrinsics) {
    this->intrinsics = intrinsics;
}

const uint16_t *model::Model::get_depth_frame() {
    if (!depth_frame) {
        throw std::runtime_error("cannot get depth frame, it is not set yet");
    }
    return this->depth_frame;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr model::Model::get_point_cloud() {
    if (!point_cloud) {
        throw std::runtime_error("cannot get pointcloud, it is not initialized.");
    }
    return this->point_cloud;
}

pcl::PointCloud<pcl::Normal>::Ptr model::Model::get_normal_cloud() {
    if (!normal_cloud) {
        throw std::runtime_error("cannot get normal cloud, it is not set.");
    }
    return this->normal_cloud;
}

const camera::ss_intrinsics *model::Model::get_intrinsics() {
    if (!intrinsics) {
        throw std::runtime_error("cannot get intrinsics, they are not set yet.");
    }
    return this->intrinsics;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr model::Model::create_point_cloud() {
    if (!depth_frame) {
        throw std::runtime_error("cannot create pointcloud, must call set_depth_frame() first.");
    }
    if (!intrinsics) {
        throw std::runtime_error("cannot create pointcloud, must call set_intrinsics() first.");
    }

    point_cloud = depth::create_point_cloud(depth_frame, intrinsics);
    return point_cloud;
}

pcl::PointCloud<pcl::Normal>::Ptr model::Model::estimate_normal_cloud() {
    if (!point_cloud) {
        throw std::runtime_error("cannot create normals, pointcloud not set.");
    }
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(point_cloud);
    ne.compute(*normals);

    normal_cloud = normals;
    return normals;
}

void model::Model::to_file(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                           const std::string &name,
                           CloudType::Type cloud_type) {
    fileHandler.save_cloud(cloud, name, cloud_type);

}

model::Model::~Model() {
    std::cout << "calling model destructor \n";;
}

