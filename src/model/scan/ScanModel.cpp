#include "ScanModel.h"
#include "Logger.h"
#include <filesystem>


namespace fs = std::filesystem;

model::ScanModel::ScanModel() :
        file_handler(), logger(logger::get_file_logger()) {
    logger::set_file_logger_location(file_handler.get_scan_path() + "info/log.txt");
}

void model::ScanModel::set_scan(const std::string &scan_name) {
    file_handler.set_scan(scan_name);
    // set the new scan path for the logger!!
    logger::set_file_logger_location(file_handler.get_scan_path() + "info/log.txt");
    logger->info("Set scan to " + scan_name);
}

void model::ScanModel::save_cloud(const std::string &cloud_name, const CloudType::Type &cloud_type) {
    auto cloud = clouds[clouds_map[cloud_name]];
    file_handler.save_cloud(cloud, cloud_name, cloud_type);
    logger->info("saved cloud " + cloud_name + "| type:" + CloudType::String(cloud_type));
}

void model::ScanModel::save_cloud(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud,
                                  const std::string &cloud_name,
                                  const CloudType::Type &cloud_type) {
    file_handler.save_cloud(cloud, cloud_name, cloud_type);
    logger->info("saved cloud " + cloud_name + "| type:" + CloudType::String(cloud_type));
}

// TODO: later add functionality where you can set whatever calibration you want to use
// this applies to the processing model not scan model!!
void model::ScanModel::update_info_json(int deg, int num_rot) {
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%m-%d-%Y %H:%M:%S");
    auto date = oss.str();

    fs::path latest_cal_path = file_handler.find_latest_calibration();
    std::string info_json_path = latest_cal_path.string()
                                 + "/" + latest_cal_path.filename().string() + ".json";;
    file_handler.update_info_json(date, deg, num_rot, info_json_path);
}


