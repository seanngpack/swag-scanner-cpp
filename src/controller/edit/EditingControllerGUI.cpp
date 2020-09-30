#include "EditingControllerGUI.h"
#include "ProcessingModel.h"
#include "SwagGUI.h"
#include "CloudType.h"

controller::EditingControllerGUI::EditingControllerGUI(std::shared_ptr<model::ProcessingModel> model,
                                                       std::shared_ptr<SwagGUI> gui) :
        model(std::move(model)),
        IControllerGUI(std::move(gui)) {}

void controller::EditingControllerGUI::run() {
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud = model->load_cloud("REGISTERED",
                                                                              CloudType::Type::REGISTERED);
    gui->display_cloud(cloud);
}

void controller::EditingControllerGUI::set_cloud_path(const std::string &path) {
    cloud_path = path;
}

void controller::EditingControllerGUI::update(const IFormsPayload &payload) {
    // do nothing for now.
}
