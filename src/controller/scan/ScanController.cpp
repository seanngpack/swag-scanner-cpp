#include "ScanController.h"
#include "Model.h"
#include "Arduino.h"
#include "SR305.h"
#include "Visualizer.h"
#include "ScanFileHandler.h"
#include <utility>

controller::ScanController::ScanController(std::shared_ptr<camera::ICamera> camera,
                                           std::shared_ptr<arduino::Arduino> arduino,
                                           std::shared_ptr<model::Model> model,
                                           std::shared_ptr<file::ScanFileHandler> file_handler) :
        camera(std::move(camera)), arduino(std::move(arduino)), model(std::move(model)),
        file_handler(std::move(file_handler)) {}

void controller::ScanController::run() {
    scan();
}

void controller::ScanController::set_deg(int deg) {
    deg = deg;
}

void controller::ScanController::set_num_rot(int num_rot) {
    num_rot = num_rot;
}


void controller::ScanController::scan() {
    update_json_time();
    camera->scan();
    const camera::intrinsics intrin = camera->get_intrinsics();
    const camera::intrinsics intrin_filt = camera->get_intrinsics_processed();
    std::cout << "started scanning..." << std::endl;
    for (int i = 0; i < num_rot; i++) {
        std::string name = std::to_string(i * deg) + ".pcd";
        camera->scan();
        std::vector<uint16_t> depth_frame_raw = camera->get_depth_frame();
        std::vector<uint16_t> depth_frame_filt = camera->get_depth_frame_processed();
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_raw = model->create_point_cloud(depth_frame_raw, intrin);
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_filt = model->create_point_cloud(depth_frame_filt,
                                                                                               intrin_filt);
        file_handler->save_cloud(cloud_raw, name, CloudType::Type::RAW);
        file_handler->save_cloud(cloud_filt, name, CloudType::Type::FILTERED);
        arduino->rotate_by(deg);
    }
}

void controller::ScanController::update_json_time() {
    // get current time
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%m-%d-%Y %H:%M:%S");
    auto str = oss.str();

    std::string info_json_path =
            file_handler->find_latest_calibration().string() + "/" + file_handler->get_scan_name() + ".json";;
    file_handler->update_info_json(str, deg, info_json_path);
}



