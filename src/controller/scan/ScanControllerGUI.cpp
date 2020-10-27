#include "ScanControllerGUI.h"
#include "FormsPayload.h"
#include "ScanFileHandler.h"
#include "SR305.h"
#include "ScanModel.h"
#include "Arduino.h"
#include "Logger.h"
#include <thread>

controller::ScanControllerGUI::ScanControllerGUI(std::shared_ptr<camera::ICamera> camera,
                                                 std::shared_ptr<arduino::Arduino> arduino,
                                                 std::shared_ptr<model::ScanModel> model,
                                                 std::shared_ptr<SwagGUI> gui) :
        ScanController(std::move(camera),
                       std::move(arduino),
                       std::move(model)),
        IControllerGUI(std::move(gui)) {}

void controller::ScanControllerGUI::run() {
    model->update_info_json(deg, num_rot);

    const camera::intrinsics intrin = camera->get_intrinsics();
    emit update_console("Started scanning...");

    if (num_rot == 0) {
        camera->scan();
        std::vector<uint16_t> depth_frame_raw = camera->get_depth_frame();
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_raw = camera->create_point_cloud(depth_frame_raw, intrin);
        model->add_cloud(cloud_raw, "0.pcd");
        model->save_cloud("0.pcd", CloudType::Type::RAW);
    }

    logger::info("started scanning...");
    for (int i = 0; i < num_rot; i++) {
        std::string name = std::to_string(i * deg) + ".pcd";
        // add delay to avoid ghosting
        std::chrono::milliseconds timespan(500);
        std::this_thread::sleep_for(timespan);
        camera->scan();
        std::vector<uint16_t> depth_frame_raw = camera->get_depth_frame();
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_raw = camera->create_point_cloud(depth_frame_raw, intrin);
        model->add_cloud(cloud_raw, name);
        model->save_cloud(name, CloudType::Type::RAW);
        arduino->rotate_by(deg);
        // add a delay to avoid ghosting
        std::this_thread::sleep_for(timespan);
    }
    emit update_console("Scan complete!");
}

void controller::ScanControllerGUI::update(const IFormsPayload &payload) {
    const auto &p = dynamic_cast<const FormsPayload &>(payload);
    model->set_scan(p.name);
    this->deg = p.angle;
    this->num_rot = p.rotations;
    // careful, don't use set_deg()
}


