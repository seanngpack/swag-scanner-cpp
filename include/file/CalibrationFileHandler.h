#ifndef SWAG_SCANNER_CALIBRATIONFILEHANDLER_H
#define SWAG_SCANNER_CALIBRATIONFILEHANDLER_H

#include "FileHandler.h"
#include "Normal.h"
#include "Point.h"
#include "json.hpp"

namespace file {

    class CalibrationFileHandler : public FileHandler {
    public:

        /**
         * Construstor creates a file handler object. Will point to the latest calibration.
         */
        CalibrationFileHandler();

        /**
         * If the create flag is true, then a new calibration scan folder will be created
         * by alphanumeric order.
         * @param auto_create_flag true if you want to automatically create a new scan folder.
         */
        CalibrationFileHandler(bool auto_create_flag);

        /**
         * Points to the given calibration scan name. If the calibration does not exist then
         * create the folder.
         * @param scan_name calibration scan that you want to point to.
         */
        CalibrationFileHandler(const char *scan_name);

        void save_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                        const std::string &cloud_name,
                        CloudType::Type cloud_type) override;

        void load_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                        const std::string &cloud_name,
                        CloudType::Type cloud_type) override;

        void load_clouds(
                std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr,
                        Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ>::Ptr> > &cloud_vector,
                CloudType::Type cloud_type) override;

        std::string get_scan_name() override;

        void set_scan_name(const std::string &scan_name) override;

        /**
         * Update the calibration .json file in the current scan folder with the
         * axis direction and center point.
         * @param dir axis direction.
         * @param pt center point.
         */
        void update_calibration_json(equations::Normal dir, equations::Point pt);

    private:
        /**
         * Get the calibration file for the current calibration scan.
         * @return the calibration .json file.
         */
        nlohmann::json get_calibration_json();
    };
}

#endif //SWAG_SCANNER_CALIBRATIONFILEHANDLER_H
