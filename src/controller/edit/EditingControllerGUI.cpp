#include "EditingControllerGUI.h"
#include "ProcessingModel.h"
#include "SwagGUI.h"
#include "CloudType.h"
#include "FormsPayload.h"
#include <filesystem>

controller::EditingControllerGUI::EditingControllerGUI(std::shared_ptr<model::ProcessingModel> model,
                                                       std::shared_ptr<SwagGUI> gui) :
        model(std::move(model)),
        IControllerGUI(std::move(gui)) {}

void controller::EditingControllerGUI::run() {
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud = model->load_cloud("REGISTERED.pcd",
                                                                              CloudType::Type::REGISTERED);
    gui->display_cloud(cloud);
}

void controller::EditingControllerGUI::update(const IFormsPayload &payload) {
    const auto &p = dynamic_cast<const FormsPayload &>(payload);
    std::filesystem::path scan_path = p.name;
    model->set_scan(scan_path.filename().string());
}
