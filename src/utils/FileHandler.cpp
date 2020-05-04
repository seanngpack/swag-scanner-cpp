#include "FileHandler.h"

using namespace boost::filesystem;

file::FileHandler::FileHandler(const std::string &all_data_folder_path, bool auto_create_flag)
        : all_data_folder_path(check_folder_input(all_data_folder_path) ? all_data_folder_path : nullptr),
          scan_folder_path(find_scan_folder(all_data_folder_path)) {
    // create subfolder (ex. raw, filtered, etc in the scanner folder
    if (auto_create_flag) {
        create_directory(scan_folder_path);
        create_sub_folders();
    }
}

void file::FileHandler::set_scan_folder_path(const std::string &path) {
    check_folder_input(path);
    this->scan_folder_path = path;
    create_sub_folders();
}

std::string file::FileHandler::get_scan_folder_path() {
    return this->scan_folder_path;
}

void file::FileHandler::save_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                   const std::string &cloud_name,
                                   CloudType cloud_type) {

    std::string out_path = scan_folder_path
                           + type_path_map.at(cloud_type)
                           + "/" + cloud_name + ".pcd";

    pcl::io::savePCDFileASCII(out_path, *cloud);
}

void file::FileHandler::open_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                   const std::string &cloud_name,
                                   CloudType cloud_type) {
    check_file_input(cloud_name);
    std::string open_path = scan_folder_path
                            + type_path_map.at(cloud_type)
                            + "/" + cloud_name;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(open_path, *cloud) == -1) {
        PCL_ERROR ("Couldn't read file \n");
    }
}

std::string file::FileHandler::find_scan_folder(const std::string &folder) {
    check_folder_input(folder);

    // if folder is empty let's start at 1.
    if (is_empty(folder)) {
        std::string name = folder + "/1";
        scan_folder_path = name;
        return name;
    }

    // make a vector to hold paths of all FOLDERS in the path
    std::vector<path> v;
    // don't include directories with '.' or without numbers in them.
    for (auto &&x : directory_iterator(folder)) {
        if (x.path().string().find('.') == std::string::npos &&
            x.path().string().find_first_of("0123456789") != std::string::npos) {
            v.push_back(x.path());
        }
    }

    // sort them using custom lambda function to order.
    std::sort(v.begin(), v.end(),
              [](auto &&path1, auto &&path2) {

                  std::string string1 = path1.string();
                  std::string string2 = path2.string();

                  // find the ending numbers of string1
                  size_t last_index = string1.find_last_not_of("0123456789");
                  std::string result1 = (string1.substr(last_index + 1));
                  // find the ending numbers of string 2
                  last_index = string2.find_last_not_of("0123456789");
                  std::string result2 = string2.substr(last_index + 1);

                  if (result1.length() == 0) {
                      return true;
                  } else if (result2.length() == 0) {
                      return true;
                  }
                  return (std::stoi(result1) < std::stoi(result2));
              }
    );


    // get the last item in list, convert to string, convert to int, then add 1
    int name_count = std::stoi(v.back().filename().string()) + 1;
    std::string name_count_str = std::to_string(name_count);

    std::string last_path = v.back().string();
    std::string to_replace = v.back().filename().string();
    std::string name = last_path.replace(last_path.find(to_replace),
                                         sizeof(name_count_str) - 1,
                                         name_count_str);
    return name;
}

void file::FileHandler::create_sub_folders() {
    for (auto element : type_path_map) {
        std::string p = scan_folder_path + element.second;
        if (!exists(p)) {
            create_directory(p);
            std::cout << "Creating folder " + p << std::endl;
        }
    }
}

bool file::FileHandler::check_folder_input(const std::string &folder) {
    if (!is_directory(folder)) {
        throw std::invalid_argument("Error, the folder path you entered does not exist.");
    }
    return true;
}

bool file::FileHandler::check_file_input(const std::string &file_path) {
    if (!exists(file_path)) {
        throw std::invalid_argument("Error, the file you provided does not exist");
    }
    return true;
}