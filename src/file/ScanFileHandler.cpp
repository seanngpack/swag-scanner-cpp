#include "ScanFileHandler.h"
#include "Logger.h"
#include <pcl/PolygonMesh.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>

namespace fs = std::filesystem;
using json = nlohmann::json;

file::ScanFileHandler::ScanFileHandler() {
    scan_folder_path = find_latest_scan();
    logger = logger::get_file_logger();
    logger::set_file_logger_location(get_scan_path() + "/info/log.txt");
}

file::ScanFileHandler::ScanFileHandler(bool auto_create_flag) {
    if (auto_create_flag) {
        auto_create_new_scan();
    } else {
        scan_folder_path = find_latest_scan().parent_path();
        scan_name = scan_folder_path.stem().string();
    }
    logger = logger::get_file_logger();
    logger::set_file_logger_location(get_scan_path() + "/info/log.txt");
}


file::ScanFileHandler::ScanFileHandler(const char *scan_name) {
    set_scan((std::string) scan_name);
    logger = logger::get_file_logger();
    logger::set_file_logger_location(get_scan_path() + "/info/log.txt");
}

void file::ScanFileHandler::auto_create_new_scan() {
    scan_folder_path = find_next_scan_folder_numeric();
    scan_name = scan_folder_path.stem().string();
    create_directory(scan_folder_path);
    create_sub_folders();
    set_swag_scanner_info_latest_scan(scan_folder_path);
}

void file::ScanFileHandler::set_scan(const std::string &scan_name) {
    if (scan_name.empty()) {
        return;
    }
    this->scan_name = scan_name;
    scan_folder_path = swag_scanner_path / "scans" / scan_name;
    if (!is_directory(scan_folder_path)) {
        create_directory(scan_folder_path);
        create_sub_folders();
        set_swag_scanner_info_latest_scan(scan_folder_path);
    }
    logger::set_file_logger_location(get_scan_path() + "/info/log.txt");
}


void file::ScanFileHandler::save_cloud(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud,
                                       const std::string &cloud_name,
                                       const CloudType::Type &cloud_type) {
    fs::path out_path = scan_folder_path / CloudType::String(cloud_type) / cloud_name;
    pcl::io::savePCDFileASCII(out_path.string(), *cloud);
    logger::info("saved cloud: " + cloud_name + " of type: " + CloudType::String(cloud_type));
}

void file::ScanFileHandler::save_mesh(const std::shared_ptr<pcl::PolygonMesh> &mesh,
                                      const std::string &mesh_name,
                                      const CloudType::Type &cloud_type) {
    fs::path out_path = scan_folder_path / CloudType::String(cloud_type) / mesh_name;
    pcl::io::saveOBJFile(out_path, *mesh);
}

std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> file::ScanFileHandler::load_cloud(const std::string &cloud_name,
                                                                                  const CloudType::Type &cloud_type) {
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    fs::path open_path = scan_folder_path / CloudType::String(cloud_type) / cloud_name;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(open_path.string(), *cloud) == -1) {
        PCL_ERROR ("Couldn't read file \n");
    }
    return cloud;
}


std::vector<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>>
file::ScanFileHandler::load_clouds(const CloudType::Type &cloud_type) {
    std::vector<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> cloud_vector;
    std::vector<fs::path> cloud_paths;

    fs::path load_path = scan_folder_path / CloudType::String(cloud_type);

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
    logger::info("loading clouds from: " + load_path.string());
    for (auto &p : cloud_paths) {
        auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(p.string(), *cloud) == -1) {
            PCL_ERROR ("Couldn't read file \n");
        }
        cloud_vector.push_back(cloud);
    }
    return cloud_vector;
}


json file::ScanFileHandler::get_info_json() {
    std::ifstream info(scan_folder_path / "info/info.json");
    json info_json;
    info >> info_json;
    return info_json;
}

void file::ScanFileHandler::update_info_json(const std::string &date,
                                             int angle,
                                             int num_rot,
                                             const std::string &cal) {
    json info_json = get_info_json();
    info_json["date"] = date;
    info_json["angle"] = angle;
    info_json["rotations"] = num_rot;
    info_json["calibration"] = cal;

    std::ofstream updated_file(scan_folder_path / "info/info.json");
    updated_file << std::setw(4) << info_json << std::endl; // write to file
}

json file::ScanFileHandler::get_calibration_json() {
    json info_json = get_info_json();
    std::string calibration_path = info_json["calibration"];
    std::ifstream calibration(calibration_path);
    json calibration_json;
    calibration >> calibration_json;
    return calibration_json;
}

fs::path file::ScanFileHandler::find_latest_scan() {
    std::ifstream info(swag_scanner_path / "settings/info.json");
    json info_json;
    info >> info_json;
    std::string latest = info_json["latest_scan"];
    logger::info("found latest scan in /settings/info.json file to be " + latest);
    return latest;
}

void file::ScanFileHandler::create_sub_folders() {
    for (const auto &element : CloudType::All) {
        fs::path p = scan_folder_path / CloudType::String(element);
        if (!exists(p) && element != CloudType::Type::CALIBRATION) {
            create_directory(p);
            logger::info("creating folder " + p.string());
        }
    }
    fs::path info_p = scan_folder_path / "info";
    if (!exists(info_p)) {
        create_directory(info_p);
        logger::info("creating folder " + info_p.string());
        std::ofstream info(scan_folder_path / "info/info.json");
        json info_json = {
                {"date",        "null"},
                {"angle",       0},
                {"rotations",   0},
                {"calibration", find_latest_calibration().string()}
        };
        info << std::setw(4) << info_json << std::endl;
    }
}


void file::ScanFileHandler::set_swag_scanner_info_latest_scan(const fs::path &folder_path) {
    std::ifstream settings(swag_scanner_path / "settings/info.json");
    json settings_json;
    settings >> settings_json;
    settings_json["latest_scan"] = folder_path.string();

    std::ofstream updated_file(swag_scanner_path / "settings/info.json");
    updated_file << std::setw(4) << settings_json << std::endl; // write to file
}
