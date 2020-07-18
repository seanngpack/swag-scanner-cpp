#include "ScanController.h"

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

    // get current time
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%m-%d-%Y %H:%M:%S");
    auto str = oss.str();

    file_handler->update_info_json(str, deg, file_handler->find_latest_calibration().string());


    const camera::ss_intrinsics intrin = camera->get_intrinsics();
    std::cout << "starting scanning..." << std::endl;
    for (int i = 0; i < num_rot; i++) {
        std::string name = std::to_string(i * deg) + ".pcd";
        std::vector<uint16_t> depth_frame = camera->get_depth_frame();
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = model->create_point_cloud(depth_frame, intrin);
        //TODO: save filtered pointcloud as well. save to /filtered I guess
        file_handler->save_cloud(cloud, name, CloudType::Type::RAW);
        arduino->rotate_table(deg);
    }
}




