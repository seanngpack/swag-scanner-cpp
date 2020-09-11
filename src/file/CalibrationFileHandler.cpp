#include "CalibrationFileHandler.h"
#include "Normal.h"
#include "Point.h"
#include <pcl/io/pcd_io.h>
#include <memory>

namespace fs = std::filesystem;
using json = nlohmann::json;

file::CalibrationFileHandler::CalibrationFileHandler() {
    std::cout << "calibration file handler constructor called" << std::endl;
    scan_folder_path = find_latest_calibration();
    scan_name = scan_folder_path.stem().string();
}

file::CalibrationFileHandler::CalibrationFileHandler(bool auto_create_flag) {
    if (auto_create_flag) {
        auto_create_new_calibration();
    } else {
        scan_folder_path = find_latest_calibration();
        scan_name = scan_folder_path.stem().string();
    }
}

file::CalibrationFileHandler::CalibrationFileHandler(const char *scan_name) {
    set_calibration((std::string) scan_name);
}

void file::CalibrationFileHandler::auto_create_new_calibration() {
    scan_folder_path = find_next_scan_folder_numeric(CloudType::Type::CALIBRATION);
    scan_name = scan_folder_path.stem().string();
    create_directory(scan_folder_path);
}

void file::CalibrationFileHandler::set_calibration(const std::string &cal_name) {
    scan_name = cal_name;
    scan_folder_path = swag_scanner_path / "calibration" / cal_name;
    if (!is_directory(scan_folder_path)) {
        create_directory(scan_folder_path);
        create_calibration_json();
    }
}

void file::CalibrationFileHandler::save_cloud(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud,
                                              const std::string &cloud_name,
                                              const CloudType::Type &cloud_type) {
    std::cout << "saving file to ";
    fs::path out_path = scan_folder_path / cloud_name;
    std::cout << out_path << std::endl;
    pcl::io::savePCDFileASCII(out_path.string(), *cloud);
}

std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> file::CalibrationFileHandler::load_cloud(const std::string &cloud_name,
                                                                                         const CloudType::Type &cloud_type) {
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud;
    fs::path open_path = scan_folder_path / cloud_name;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(open_path.string(), *cloud) == -1) {
        PCL_ERROR ("Couldn't read file \n");
    }
    return cloud;
}

std::vector<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> file::CalibrationFileHandler::load_clouds(
        const CloudType::Type &cloud_type) {
    std::vector<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> cloud_vector;
    std::vector<fs::path> cloud_paths;
    fs::path load_path = scan_folder_path;

    // load paths into cloud_paths vector
    for (const auto &p : fs::directory_iterator(load_path)) {
        // extension must be .pcd and must have number in the filename
        if (p.path().extension() == ".pcd" && p.path().string().find_first_of("0123456789") != std::string::npos) {
            cloud_paths.push_back(p.path());
        }
    }

    // sort the paths numerically
    std::sort(cloud_paths.begin(), cloud_paths.end(), path_sort);

    // finally we load the clouds into the cloud_vector
    for (auto &p : cloud_paths) {
        auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(p.string(), *cloud) == -1) {
            PCL_ERROR ("Couldn't read file \n");
        }
        std::cout << "loading " << p.string() << std::endl;
        cloud_vector.push_back(cloud);
    }
    std::cout << "finished loading clouds" << std::endl;
    return cloud_vector;
}

std::string file::CalibrationFileHandler::get_scan_name() {
    return this->scan_name;
}


void file::CalibrationFileHandler::update_calibration_json(const equations::Normal &dir, const equations::Point &pt) {
    json calibration_json = get_calibration_json();
    calibration_json["axis_direction"] = {dir.A, dir.B, dir.C};
    calibration_json["origin_point"] = {pt.x, pt.y, pt.z};

    std::string calibration_path = scan_folder_path.string() + "/" + scan_name + ".json";
    std::ofstream updated_file(calibration_path);
    updated_file << std::setw(4) << calibration_json << std::endl; // write to file
}

void file::CalibrationFileHandler::create_calibration_json() {
    std::string calibration_path = scan_folder_path.string() + "/" + scan_name + ".json";
    std::ofstream calibration(calibration_path); // create json file
    json calibration_json = {
            {"origin_point",   {0.0, 0.0, 0.0}},
            {"axis_direction", {0.0, 0.0, 0.0}}
    };
    calibration << std::setw(4) << calibration_json << std::endl; // write to file
}

json file::CalibrationFileHandler::get_calibration_json() {
    std::string calibration_path = scan_folder_path.string() + "/" + scan_name + ".json";
    std::ifstream calibration(calibration_path);
    json calibration_json;
    calibration >> calibration_json;
    return calibration_json;
}


