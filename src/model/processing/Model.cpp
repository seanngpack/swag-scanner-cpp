#include "Model.h"
#include "Algorithms.h"

model::Model::Model() {
    depth_frame = nullptr;
    point_cloud = nullptr;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr model::Model::create_point_cloud() {
    if (!depth_frame) {
        throw std::runtime_error("cannot create pointcloud, must call set_depth_frame() first.");
    }
    point_cloud = algos::create_point_cloud(depth_frame,);
    return point_cloud;
}

void model::Model::set_depth_frame(const uint16_t *depth_frame) {
    this->depth_frame = depth_frame;
}
