#include "CalibrationControllerGUI.h"
#include "CalibrationFileHandler.h"
#include "Model.h"
#include "Normal.h"
#include "Point.h"
#include "SR305.h"
#include "Arduino.h"
#include "Visualizer.h"
#include "SwagGUI.h"
#include "FormsPayload.h"
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
        IControllerGUI(std::move(gui)) {}


void controller::CalibrationControllerGUI::run() {
    std::cout << "number of deg is: " << deg << std::endl;
    std::cout << "number of rot is: " << num_rot << std::endl;

    emit update_console("Starting scan");
    scan();
    emit update_console("Scan complete");
    emit update_console("Loading clouds...");
    std::vector<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> cloud_vector = file_handler->load_clouds(
            CloudType::Type::CALIBRATION);
    emit update_console("Load complete");
    emit update_console("Performing calculations...");
    equations::Normal axis_dir = model->calculate_axis_dir(ground_planes);
    equations::Point center = model->calculate_center_pt(axis_dir, upright_planes);
    emit update_console("Calculations complete. Latest calibration updated.");
    file_handler->update_calibration_json(axis_dir, center);
    emit update_console("Returning to home position");
    arduino->rotate_to(0);

//    viewer->ptVis(cloud_vector[0], pcl::PointXYZ(center.x, center.y, center.z));
}


void controller::CalibrationControllerGUI::update(const IFormsPayload &payload) {
    const auto &p = dynamic_cast<const FormsPayload &>(payload);
    file_handler->set_calibration(p.name);
    set_deg(p.deg);
    set_num_rot(p.rot);
}



//TODO: at some point do some checking to make sure deg and rot is not 0, if they are then use default values