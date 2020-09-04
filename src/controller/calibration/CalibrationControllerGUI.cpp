#include "CalibrationControllerGUI.h"
#include "Model.h"
#include "CalibrationFileHandler.h"
#include "Normal.h"
#include "Point.h"
#include "SR305.h"
#include "Arduino.h"
#include "Visualizer.h"
#include "SwagGUI.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

controller::CalibrationControllerGUI::CalibrationControllerGUI(std::shared_ptr<camera::ICamera> camera,
                                                               std::shared_ptr<arduino::Arduino> arduino,
                                                               std::shared_ptr<model::Model> model,
                                                               std::shared_ptr<file::CalibrationFileHandler> file_handler,
                                                               std::shared_ptr<visual::Visualizer> viewer,
                                                               std::shared_ptr<SwagGUI> gui) :
        CalibrationController(std::move(camera),
                              std::move(arduino),
                              std::move(model),
                              std::move(file_handler),
                              std::move(viewer)),
        gui(std::move(gui)) {}

void controller::CalibrationControllerGUI::run() {
    std::cout << "gets ehre" << std::endl;
    get_name();
    std::cout << "finished getting name" << std::endl;
    get_deg();
    std::cout << "finished getting deg" << std::endl;
    get_rot();
    std::cout << "prints the getters just fine" << get_rot() << std::endl;

    update_console("Starting scan");
    scan();
    update_console("Scan complete");
    update_console("Loading clouds...");
    std::vector<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> cloud_vector = file_handler->load_clouds(
            CloudType::Type::CALIBRATION);
    update_console("Load complete");
    update_console("Performing calculations...");
    equations::Normal axis_dir = model->calculate_axis_dir(ground_planes);
    equations::Point center = model->calculate_center_pt(axis_dir, upright_planes);
    update_console("Calculations complete. Latest calibration updated.");
    file_handler->update_calibration_json(axis_dir, center);
    update_console("Returning to home position");
    arduino->rotate_to(0);

    viewer->ptVis(cloud_vector[0], pcl::PointXYZ(center.x, center.y, center.z));
}

std::string controller::CalibrationControllerGUI::get_name() {
    gui->update_name();
}

int controller::CalibrationControllerGUI::get_deg() {
    gui->update_deg();
}

int controller::CalibrationControllerGUI::get_rot() {
    gui->update_rot();
}

void controller::CalibrationControllerGUI::update_console(const std::string &info) {
    gui->update_console(info);
}



//TODO: at some point do some checking to make sure deg and rot is not 0, if they are then use default values