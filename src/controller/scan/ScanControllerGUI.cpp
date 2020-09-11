#include "ScanControllerGUI.h"
#include "FormsPayload.h"
#include "ScanFileHandler.h"
#include "SR305.h"
#include "Model.h"
#include "Arduino.h"

controller::ScanControllerGUI::ScanControllerGUI(std::shared_ptr<camera::ICamera> camera,
                                                 std::shared_ptr<arduino::Arduino> arduino,
                                                 std::shared_ptr<model::Model> model,
                                                 std::shared_ptr<file::ScanFileHandler> file_handler,
                                                 std::shared_ptr<SwagGUI> gui) :
        ScanController(std::move(camera),
                       std::move(arduino),
                       std::move(model),
                       std::move(file_handler)),
        IControllerGUI(std::move(gui)) {}

void controller::ScanControllerGUI::run() {
    update_json_time();


    const camera::intrinsics intrin = camera->get_intrinsics();
    const camera::intrinsics intrin_filt = camera->get_intrinsics_processed();
    emit update_console("Started scanning...");

    if (num_rot == 0) {
        camera->scan();
        std::vector<uint16_t> depth_frame_raw = camera->get_depth_frame();
        std::vector<uint16_t> depth_frame_filt = camera->get_depth_frame_processed();
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_raw = model->create_point_cloud(depth_frame_raw, intrin);
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_filt = model->create_point_cloud(depth_frame_filt,
                                                                                               intrin_filt);
        file_handler->save_cloud(cloud_raw, "0.pcd", CloudType::Type::RAW);
        file_handler->save_cloud(cloud_filt, "0.pcd", CloudType::Type::FILTERED);
    }

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
    emit update_console("Scan complete!");
}

void controller::ScanControllerGUI::update(const IFormsPayload &payload) {
    const auto &p = dynamic_cast<const FormsPayload &>(payload);
    file_handler->set_scan(p.name);
    this->deg = p.deg;
    this->num_rot = p.rot;
    // careful, don't use set_deg()
}


