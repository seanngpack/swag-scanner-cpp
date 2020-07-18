#include "FilterTestingController.h"

controller::FilterTestingController::FilterTestingController(std::shared_ptr<camera::ICamera> camera,
                                                             std::shared_ptr<arduino::Arduino> arduino,
                                                             std::shared_ptr<model::Model> model,
                                                             std::shared_ptr<file::ScanFileHandler> file_handler,
                                                             std::shared_ptr<visual::Visualizer> viewer) :
        camera(std::move(camera)), arduino(std::move(arduino)), model(std::move(model)),
        file_handler(std::move(file_handler)), viewer(std::move(viewer)) {}


void controller::FilterTestingController::run() {
    const camera::ss_intrinsics *intrin = camera->get_intrinsics();
    camera->scan();
    std::vector<uint16_t> depth_frame = camera->get_depth_frame();
    std::vector<uint16_t> depth_frame_processed = camera->get_depth_frame_processed();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 = model->create_point_cloud(depth_frame, intrin);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 = model->create_point_cloud(depth_frame_processed, intrin);
    viewer->compareVis(cloud1, cloud2);
}
