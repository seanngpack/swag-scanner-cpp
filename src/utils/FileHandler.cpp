#include "FileHandler.h"

using namespace boost::filesystem;

file::FileHandler::FileHandler(const std::string &all_data_folder_path, bool auto_create_flag)
        : all_data_folder_path(check_folder_input(all_data_folder_path) ? all_data_folder_path : nullptr),
          scan_folder_path(find_scan_folder(all_data_folder_path)),
          auto_create_flag(auto_create_flag) {
    // create subfolder (ex. raw, filtered, etc in the scanner folder
    if (auto_create_flag) {
        create_directory(scan_folder_path);
        create_sub_folders();
    }
}

file::FileHandler::FileHandler(bool auto_create_flag) :
        all_data_folder_path(default_data_path),
        scan_folder_path(find_scan_folder(all_data_folder_path)),
        auto_create_flag(auto_create_flag) {

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

void file::FileHandler::save_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                   const std::string &cloud_name,
                                   CloudType::Type cloud_type) {

    std::cout << "saving file to ";
    std::string out_path = scan_folder_path
                           + "/"
                           + CloudType::String(cloud_type)
                           + "/" + cloud_name + ".pcd";
    std::cout << out_path << std::endl;

    pcl::io::savePCDFileASCII(out_path, *cloud);
}

void file::FileHandler::load_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                   const std::string &cloud_name,
                                   CloudType::Type cloud_type) {
    check_file_input(cloud_name);
    std::string open_path = scan_folder_path
                            + "/"
                            + CloudType::String(cloud_type)
                            + "/" + cloud_name;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(open_path, *cloud) == -1) {
        PCL_ERROR ("Couldn't read file \n");
    }
}

void file::FileHandler::load_clouds(
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ>::Ptr>> &cloud_vector,
        CloudType::Type cloud_type,
        const std::string &folder_path) {
    if (folder_path == "" && !auto_create_flag) {
        throw std::runtime_error("Error, auto folder creation is not set so you must"
                                 "enter a valid folder path when loading clouds");
    }
    check_folder_input(folder_path);
    std::string load_path;
    std::vector<path> cloud_paths;
    if (folder_path.empty()) {
        load_path = folder_path + "/" + CloudType::String(cloud_type);
    } else {
        load_path = scan_folder_path + "/" + CloudType::String(cloud_type);;
    }
    check_folder_input(load_path);

    // load paths into cloud_paths vector
    for (auto &p : boost::filesystem::directory_iterator(load_path)) {
        // extension must be .pcd and must have number in the filename
        if (p.path().extension() == ".pcd" && p.path().string().find_first_of("0123456789") != std::string::npos) {
            cloud_paths.push_back(p.path());
        }
    }

    // sort the paths numerically
    std::sort(cloud_paths.begin(), cloud_paths.end(), path_sort);

    for (auto &p : cloud_paths) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(p.string(), *cloud) == -1) {
            PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        }
        std::cout << p.string() << std::endl;
        cloud_vector.push_back(cloud);
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
    std::sort(v.begin(), v.end(), path_sort);

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
    for (const auto element : CloudType::All) {
        std::string p = scan_folder_path + "/" + CloudType::String(element);
        if (!exists(p)) {
            create_directory(p);
            std::cout << "Creating folder " + p << std::endl;
        }
    }
}

bool file::FileHandler::check_folder_input(const std::string &folder) {
    if (!is_directory(folder)) {
        throw std::invalid_argument("Folder path error, " + folder + " does not exist.");
    }
    return true;
}

bool file::FileHandler::check_file_input(const std::string &file_path) {
    if (!exists(file_path)) {
        throw std::invalid_argument("File path error, " + file_path + " does not exist");
    }
    return true;
}


