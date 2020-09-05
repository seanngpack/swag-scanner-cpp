#include "IFileHandler.h"
#include <CoreServices/CoreServices.h>

using namespace boost::filesystem;
using json = nlohmann::json;

path file::IFileHandler::swag_scanner_path = []() {
    FSRef ref;
    OSType folderType = kApplicationSupportFolderType;
    char path[PATH_MAX];

    FSFindFolder(kUserDomain, folderType, kCreateFolder, &ref);
    FSRefMakePath(&ref, (UInt8 *) &path, PATH_MAX);
    boost::filesystem::path program_folder = "/SwagScanner";
    program_folder = path / program_folder;
    return program_folder;
}();

json file::IFileHandler::load_settings_json() {
    std::string settings_path = swag_scanner_path.string() + "/settings/settings.json";
    std::ifstream settings(settings_path);
    json settings_json;
    settings >> settings_json;
    return settings_json;
}

void file::IFileHandler::write_settings_json(const json &j) {
    std::string settings_path = swag_scanner_path.string() + "/settings/settings.json";
    std::ofstream updated_file(settings_path);
    updated_file << std::setw(4) << j << std::endl; // write to file
}

path file::IFileHandler::find_latest_calibration() {
    std::string calibrations_folder = swag_scanner_path.string() + "/calibration";
    typedef std::multimap<std::time_t, boost::filesystem::path> result_set_t;
    result_set_t result_set;

    // store folders in ascending order
    if (boost::filesystem::exists(calibrations_folder) && boost::filesystem::is_directory(calibrations_folder)) {
        for (auto &&x : boost::filesystem::directory_iterator(calibrations_folder)) {
            if (is_directory(x) && x.path().filename() != ".DS_Store") {
                result_set.insert(result_set_t::value_type(last_write_time(x.path()), x.path()));
            }
        }
    }
    // get the last element which is the latest date
    // gives the path to the .json file inside the folder so that's why it's dirty
    boost::filesystem::path p =
            boost::filesystem::path(result_set.rbegin()->second.string() + "/" +
                                    result_set.rbegin()->second.filename().string() + ".json");
    return p;
}

bool file::IFileHandler::path_sort(const boost::filesystem::path &path1, const boost::filesystem::path &path2) {
    std::string string1 = path1.string();
    std::string string2 = path2.string();

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

path file::IFileHandler::find_next_scan_folder_numeric(const CloudType::Type &type) {
    boost::filesystem::path folder = swag_scanner_path / "/scans";
    if (type == CloudType::Type::CALIBRATION) {
        folder = swag_scanner_path / "calibration";
    }

    if (!is_directory(folder)) {
        throw std::invalid_argument("This shouldn't happen");
    }

    // if folder is empty let's start at 1.
    if (is_empty(folder)) {
        boost::filesystem::path name = folder / "/1";
        return name;
    }

    // make a vector to hold paths of all FOLDERS in the path
    std::vector<boost::filesystem::path> v;
    // don't include directories with '.' or without numbers in them.
    for (auto &&x : boost::filesystem::directory_iterator(folder)) {
        if (x.path().string().find('.') == std::string::npos &&
            x.path().string().find_first_of("0123456789") != std::string::npos) {
            v.push_back(x.path());
        }
    }

    // if the vector is 0 that means there are no folders with numeric names
    if (v.empty()) {
        boost::filesystem::path name = folder / "/1";
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
    return boost::filesystem::path(name);
}

path file::IFileHandler::find_next_scan_folder_numeric() {
    find_next_scan_folder_numeric(CloudType::Type::NONE);
}


std::vector<std::string> file::IFileHandler::get_all_scans() {
    boost::filesystem::path scans_folder = swag_scanner_path / "scans";
    std::vector<std::string> scans;
    for (auto &&x : boost::filesystem::directory_iterator(scans_folder)) {
        if (is_directory(x) && x.path().filename() != ".DS_Store") {
            scans.push_back(x.path().filename().string());
        }
    }
    return scans;
}


std::vector<std::string> file::IFileHandler::get_all_calibrations() {
    boost::filesystem::path calibrations_folder = swag_scanner_path / "calibration";
    std::vector<std::string> calibrations;
    for (auto &&x : boost::filesystem::directory_iterator(calibrations_folder)) {
        if (is_directory(x) && x.path().filename() != ".DS_Store") {
            calibrations.push_back(x.path().filename().string());
        }
    }
    return calibrations;
}
