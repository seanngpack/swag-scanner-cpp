#include "CalibrationController.h"
#include "CalibrationFileHandler.h"
#include "CalibrationModel.h"
#include "Constants.h"
#include "Normal.h"
#include "Plane.h"
#include "Point.h"
#include "SR305.h"
#include "Arduino.h"
#include "Visualizer.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <memory>


controller::CalibrationController::CalibrationController(std::shared_ptr<camera::ICamera> camera,
                                                         std::shared_ptr<arduino::Arduino> arduino,
                                                         std::shared_ptr<model::CalibrationModel> model) :
        camera(std::move(camera)), arduino(std::move(arduino)), model(std::move(model)) {}

void controller::CalibrationController::run() {
    scan();

    model->calculate_center_point();
    model->refine_center_point();
    model->update_calibration_json();
//    viewer->ptVis(cloud_vector[0], pcl::PointXYZ(center.x, center.y, center.z));
}

void controller::CalibrationController::scan() {
    using namespace constants;

    const camera::intrinsics intrin = camera->get_intrinsics_processed();
    for (int i = 0; i < num_rot; i++) {
        std::string cloud_name = std::to_string(i * deg) + ".pcd";
        camera->scan();
        std::vector<uint16_t> depth_frame = camera->get_depth_frame_processed();
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud = camera->create_point_cloud(depth_frame, intrin);

        // todo: move crop and voxel to IModel.h, i guess all models should access to these methods lol
        // crop and voxel should be in-place
        // parameter should be cloud name
        model->crop_cloud(cloud, cal_min_x, cal_max_x, cal_min_y, cal_max_y, cal_min_z, cal_max_z);
        model->voxel_grid_filter(cloud, .001);
        model->add_cloud(cloud, cloud_name);
        model->save_cloud(cloud_name);

        arduino->rotate_by(deg);
    }
    arduino->rotate_to(0);

}

void controller::CalibrationController::set_deg(int deg) {
    this->deg = deg;
}

void controller::CalibrationController::set_num_rot(int num_rot) {
    this->num_rot = num_rot;
}



