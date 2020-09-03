#include "FilterTestingController.h"
#include "Arduino.h"
#include "Model.h"
#include "ScanFileHandler.h"
#include "Visualizer.h"
#include "SR305.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

controller::FilterTestingController::FilterTestingController(std::shared_ptr<camera::SR305> camera,
                                                             std::shared_ptr<model::Model> model,
                                                             std::shared_ptr<file::ScanFileHandler> file_handler,
                                                             std::shared_ptr<visual::Visualizer> viewer) :
        camera(std::move(camera)), model(std::move(model)),
        file_handler(std::move(file_handler)), viewer(std::move(viewer)) {}


void controller::FilterTestingController::run() {
    const camera::intrinsics intrin = camera->get_intrinsics();
    camera->scan();
    std::vector<uint16_t> depth_frame = camera->get_depth_frame();
    std::vector<uint16_t> depth_frame_processed = camera->get_depth_frame_processed();
    const camera::intrinsics intrin2 = camera->get_intrinsics_processed();
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud1 = model->create_point_cloud(depth_frame, intrin);
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud2 = model->create_point_cloud(depth_frame_processed, intrin2);
    viewer->compareVis(cloud1, cloud2);
}
