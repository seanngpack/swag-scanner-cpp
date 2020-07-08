#include "CalibrationFileHandler.h"

using namespace boost::filesystem;

void file::CalibrationFileHandler::save_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                         const std::string &cloud_name, CloudType::Type cloud_type) {
    std::cout << "saving file to ";
    path out_path = scan_folder_path / "/" / cloud_name / ".pcd";
    std::cout << out_path << std::endl;
    pcl::io::savePCDFileASCII(out_path.string(), *cloud);
}

void file::CalibrationFileHandler::load_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                              const std::string &cloud_name, CloudType::Type cloud_type) {
    path open_path = swag_scanner_path / "/calibration/" / scan_name / "/" / cloud_name;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(open_path.string(), *cloud) == -1) {
        PCL_ERROR ("Couldn't read file \n");
    }
}

void file::CalibrationFileHandler::load_clouds(
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ>::Ptr>> &cloud_vector,
        CloudType::Type cloud_type) {
    std::vector<path> cloud_paths;
    path load_path = scan_folder_path / "/" / scan_name / ".json";

    // load paths into cloud_paths vector
    for (auto &p : directory_iterator(load_path)) {
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

std::string file::CalibrationFileHandler::get_scan_name() {
    return this->scan_name;
}

void file::CalibrationFileHandler::set_scan_name(const std::string &scan_name) {
    this->scan_name = scan_name;
}
