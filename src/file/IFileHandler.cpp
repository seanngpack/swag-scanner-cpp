#include "IFileHandler.h"
#include <CoreServices/CoreServices.h>
#include <fstream>

namespace fs = std::filesystem;
using json = nlohmann::json;

fs::path file::IFileHandler::swag_scanner_path = []() {
    FSRef ref;
    OSType folderType = kApplicationSupportFolderType;
    char mac_path[PATH_MAX];

    FSFindFolder(kUserDomain, folderType, kCreateFolder, &ref);
    FSRefMakePath(&ref, (UInt8 *) &mac_path, PATH_MAX);
    fs::path program_folder = fs::path(mac_path) / "SwagScanner";
    return program_folder;
}();


bool file::IFileHandler::check_program_folder() {
    if (!exists(swag_scanner_path)) {
        std::cout << "No SwagScanner application folder detected, creating one at: " + swag_scanner_path.string()
                  << std::endl;
        create_directory(swag_scanner_path);
        create_directory(swag_scanner_path / "settings");
        create_directory(swag_scanner_path / "scans");
        create_directory(swag_scanner_path / "calibration");
        create_directory(swag_scanner_path / "calibration/default_calibration");
        // create info JSON
        std::string info_path = swag_scanner_path / "settings/info.json";
        std::ofstream info(info_path); // create json file
        json swag_scanner_info_json = {
                {"version",          .1},
                {"latest_scan",      "none"},
                {"current_position", 0}
        };
        // create default calibration JSON
        info << std::setw(4) << swag_scanner_info_json << std::endl; // write to file
        std::ofstream calibration(
                swag_scanner_path / "calibration/default_calibration/default_calibration.json"); // create json file
        json calibration_json = {
                {"origin_point",   {-0.0002, 0.0344,  0.4294}},
                {"axis_direction", {-0.0158, -0.8661, -0.4996}}
        };
        // create config JSON
        calibration << std::setw(4) << calibration_json << std::endl; // write to file
        std::ofstream config(swag_scanner_path / "settings/config.json"); // create json file
        json config_json = {
                {"decimation_magnitude",     2},
                {"spatial_filter_magnitude", 1},
                {"spatial_smooth_alpha",     .45},
                {"spatial_smooth_delta",     5}
        };
        config << std::setw(4) << config_json << std::endl; // write to file
        return false;
    }
    return true;
}


json file::IFileHandler::load_swag_scanner_info_json() {
    std::ifstream settings(swag_scanner_path / "settings/info.json");
    json settings_json;
    settings >> settings_json;
    return settings_json;
}


void file::IFileHandler::write_swag_scanner_info_json(const json &j) {
    std::ofstream updated_file(swag_scanner_path / "settings/info.json");
    updated_file << std::setw(4) << j << std::endl; // write to file
}

nlohmann::json file::IFileHandler::get_swag_scanner_config_json() {
    std::string config_path = swag_scanner_path / "settings/config.json";
    std::ifstream config(config_path);
    json config_json;
    config >> config_json;
    return config_json;
}


fs::path file::IFileHandler::find_latest_calibration() {
    fs::path calibrations_folder_path = swag_scanner_path / "calibration";
    fs::directory_entry latest_cal;
    if (!fs::exists(calibrations_folder_path)) {
        throw std::runtime_error("error, calibrations folder does not exist");
    }
    if (fs::is_empty(calibrations_folder_path)) {
        return latest_cal.path();
    }

    for (const auto &x : fs::directory_iterator(calibrations_folder_path)) {
        if (latest_cal.path().empty()) {
            latest_cal.assign(x.path());
            continue;
        }
        if (is_directory(x) &&
            x.path().filename() != ".DS_Store" &&
            fs::last_write_time(x) > fs::last_write_time(latest_cal)) {
            latest_cal.assign(x.path());
        }
    }

    return latest_cal.path();
}


bool file::IFileHandler::path_sort(const fs::path &path1, const fs::path &path2) {
    std::string string1 = path1.string();
    std::string string2 = path2.string();

    // TODO: see if there is a better way for this
    // remove ".pcd" from files
    if (path1.has_extension()) {
        size_t idx = string1.find_last_of(".");
        string1 = string1.substr(0, idx);
    }

    if (path2.has_extension()) {
        size_t idx = string2.find_last_of(".");
        string2 = string2.substr(0, idx);
    }

    // find the ending numbers of string1
    size_t last_index = string1.find_last_not_of("0123456789");

    std::string result1 = (string1.substr(last_index + 1));
    // find the ending numbers of string 2
    last_index = string2.find_last_not_of("0123456789");
    std::string result2 = string2.substr(last_index + 1);

    if (result1.length() == 0 || result2.length() == 0) {
        return true;
    }
    return (std::stoi(result1) < std::stoi(result2));
}


fs::path file::IFileHandler::find_next_scan_folder_numeric(const CloudType::Type &type) {
    fs::path folder = swag_scanner_path / "scans";
    if (type == CloudType::Type::CALIBRATION) {
        folder = swag_scanner_path / "calibration";
    }

    if (!is_directory(folder)) {
        throw std::invalid_argument("This shouldn't happen");
    }

    // if folder is empty let's start at 1.
    if (is_empty(folder)) {
        fs::path name = folder / "/1";
        return name;
    }

    // make a vector to hold paths of all FOLDERS in the path
    std::vector<fs::path> v;
    // don't include directories with '.' or without numbers in them.
    for (auto &&x : fs::directory_iterator(folder)) {
        if (x.path().string().find('.') == std::string::npos &&
            x.path().string().find_first_of("0123456789") != std::string::npos) {
            v.push_back(x.path());
        }
    }

    // if the vector is 0 that means there are no folders with numeric names
    if (v.empty()) {
        fs::path name = folder / "/1";
        return name;
    }

    // sort them using custom lambda function to order.
    std::sort(v.begin(), v.end(), path_sort);

    // get the last item in list, convert to string, convert to int, then add 1
    int name_count = std::stoi(v.back().filename().string()) + 1;
    std::string name_count_str = std::to_string(name_count);

    std::string last_path = v.back().string();
    std::string to_replace = v.back().filename().string();
    std::string name = last_path.replace(last_path.find(to_replace),
                                         sizeof(name_count_str) - 1,
                                         name_count_str);
    return fs::path(name);
}

fs::path file::IFileHandler::find_next_scan_folder_numeric() {
    find_next_scan_folder_numeric(CloudType::Type::NONE);
}


std::vector<std::string> file::IFileHandler::get_all_scans() {
    fs::path scans_folder = swag_scanner_path / "scans";
    std::vector<std::string> scans;
    for (const auto &x : fs::directory_iterator(scans_folder)) {
        if (is_directory(x) && x.path().filename() != ".DS_Store") {
            scans.push_back(x.path().filename().string());
        }
    }
    return scans;
}


std::vector<std::string> file::IFileHandler::get_all_calibrations() {
    fs::path calibrations_folder = swag_scanner_path / "calibration";
    std::vector<std::string> calibrations;
    for (auto &&x : fs::directory_iterator(calibrations_folder)) {
        if (is_directory(x) && x.path().filename() != ".DS_Store") {
            calibrations.push_back(x.path().filename().string());
        }
    }
    return calibrations;
}



