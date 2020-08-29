#include "ScanFileHandler.h"
#include <pcl/io/pcd_io.h>

using namespace boost::filesystem;
using json = nlohmann::json;

file::ScanFileHandler::ScanFileHandler() {
    bool exists = check_program_folder();
    if (!exists) {
        scan_folder_path = find_next_scan_folder_numeric();
        scan_name = scan_folder_path.stem().string();
        create_directory(scan_folder_path);
        create_sub_folders();
        set_settings_latest_scan(scan_folder_path);
    } else {
        scan_folder_path = find_latest_scan_folder();
    }
}

file::ScanFileHandler::ScanFileHandler(bool auto_create_flag) {
    scan_folder_path = find_next_scan_folder_numeric();
    scan_name = scan_folder_path.stem().string();
    std::cout << scan_folder_path << std::endl;
    if (auto_create_flag) {
        create_directory(scan_folder_path);
        create_sub_folders();
        set_settings_latest_scan(scan_folder_path);
    }
}


file::ScanFileHandler::ScanFileHandler(const char *scan_name) {
    this->scan_name = scan_name;
    scan_folder_path = swag_scanner_path / "scans" / scan_name;
    if (!is_directory(scan_folder_path)) {
        create_directory(scan_folder_path);
        create_sub_folders();
        set_settings_latest_scan(scan_folder_path);
    }
}


void file::ScanFileHandler::save_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                       const std::string &cloud_name,
                                       CloudType::Type cloud_type) {
    std::cout << "saving file to ";
    path out_path = scan_folder_path / CloudType::String(cloud_type) / cloud_name;
    std::cout << out_path << std::endl;
    pcl::io::savePCDFileASCII(out_path.string(), *cloud);
}

void file::ScanFileHandler::load_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                       const std::string &cloud_name,
                                       CloudType::Type cloud_type) {
    path open_path = scan_folder_path / CloudType::String(cloud_type) / cloud_name;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(open_path.string(), *cloud) == -1) {
        PCL_ERROR ("Couldn't read file \n");
    }
}


void file::ScanFileHandler::load_clouds(
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ>::Ptr>> &cloud_vector,
        CloudType::Type cloud_type) {
    std::vector<path> cloud_paths;
    path load_path = scan_folder_path / CloudType::String(cloud_type);

    // load paths into cloud_paths vector
    for (auto &p : boost::filesystem::directory_iterator(load_path)) {
        // extension must be .pcd and must have number in the filename
        if (p.path().extension() == ".pcd" && p.path().string().find_first_of("0123456789") != std::string::npos) {
            cloud_paths.push_back(p.path());
        }
    }

    // sort the paths numerically
    std::sort(cloud_paths.begin(), cloud_paths.end(), path_sort);

    // finally we load the clouds into the cloud_vector
    for (auto &p : cloud_paths) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(p.string(), *cloud) == -1) {
            PCL_ERROR ("Couldn't read file \n");
        }
        std::cout << "loading " << p.string() << std::endl;
        cloud_vector.push_back(cloud);
    }
    std::cout << "finished loading clouds" << std::endl;
}

std::string file::ScanFileHandler::get_scan_name() {
    return this->scan_name;
}

void file::ScanFileHandler::set_scan_name(const std::string &scan_name) {
    this->scan_name = scan_name;
}

json file::ScanFileHandler::get_info_json() {
    std::ifstream info(scan_folder_path.string() + "/info/info.json");
    json info_json;
    info >> info_json;
    return info_json;
}

void file::ScanFileHandler::update_info_json(std::string date, int angle, std::string cal) {
    json info_json = get_info_json();
    info_json["date"] = date;
    info_json["angle"] = angle;
    info_json["calibration"] = cal;

    std::ofstream updated_file(scan_folder_path.string() + "/info/info.json");
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

path file::ScanFileHandler::find_latest_scan_folder() {
    std::ifstream settings(swag_scanner_path.string() + "/settings/settings.json");
    json settings_json;
    settings >> settings_json;
    std::string latest = settings_json["latest_scan"];
    std::cout << "found latest scan in settings.json file to be " << latest << std::endl;
    return latest;
}

bool file::ScanFileHandler::check_program_folder() {
    if (!exists(swag_scanner_path)) {
        std::cout << "No SwagScanner application folder detected, creating one at: " + swag_scanner_path.string()
                  << std::endl;
        create_directory(swag_scanner_path);
        create_directory(swag_scanner_path / "settings");
        create_directory(swag_scanner_path / "scans");
        create_directory(swag_scanner_path / "calibration");
        create_directory(swag_scanner_path / "calibration/default_calibration");
        std::ofstream settings(swag_scanner_path.string() + "/settings/settings.json"); // create json file
        json settings_json = {
                {"version",     .1},
                {"latest_scan", "none"},
                {"current_position", 0}
        };
        settings << std::setw(4) << settings_json << std::endl; // write to file
        std::ofstream calibration(
                swag_scanner_path.string() +
                "/calibration/default_calibration/default_calibration.json"); // create json file
        json calibration_json = {
                {"origin point",   {-0.0002, 0.0344,  0.4294}},
                {"axis direction", {-0.0158, -0.8661, -0.4996}}
        };
        calibration << std::setw(4) << calibration_json << std::endl; // write to file
        return false;
    }
    return true;
}


void file::ScanFileHandler::create_sub_folders() {
    for (const auto &element : CloudType::All) {
        path p = scan_folder_path / CloudType::String(element);
        if (!exists(p) && element != CloudType::Type::CALIBRATION) {
            create_directory(p);
            std::cout << "Creating folder " + p.string() << std::endl;
        }
    }
    path info_p = scan_folder_path / "info";
    if (!exists(info_p)) {
        create_directory(info_p);
        std::cout << "Creating folder " + info_p.string() << std::endl;
        std::ofstream info(scan_folder_path.string() + "/info/info.json");
        json info_json = {
                {"date",        "null"},
                {"angle",       0},
                {"calibration", find_latest_calibration().string()}
        };
        info << std::setw(4) << info_json << std::endl;
    }
}


void file::ScanFileHandler::set_settings_latest_scan(path folder_path) {
    std::ifstream settings(swag_scanner_path.string() + "/settings/settings.json");
    json settings_json;
    settings >> settings_json;
    settings_json["latest_scan"] = folder_path.string();

    std::ofstream updated_file(swag_scanner_path.string() + "/settings/settings.json");
    updated_file << std::setw(4) << settings_json << std::endl; // write to file
}
