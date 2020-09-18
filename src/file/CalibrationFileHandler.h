#ifndef SWAG_SCANNER_CALIBRATIONFILEHANDLER_H
#define SWAG_SCANNER_CALIBRATIONFILEHANDLER_H

#include "IFileHandler.h"

namespace equations {
    class Normal;

    class Point;
}

namespace file {

    class CalibrationFileHandler : public IFileHandler {
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

        /**
         * Calibration folder will be created and name will be be assigned based on previous calibrations in
         * alphanumeric order.
         */
        void auto_create_new_calibration();

        /**
         * Point to the given calibration. If the calibration does not exist then create a new one.
         * @param cal_name name of the calibration.
         */
        void set_calibration(const std::string &cal_name);

        // TODO: Later refactor this and load calibration... They don't need to take in CloudTypes
        void save_cloud(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud,
                        const std::string &cloud_name,
                        const CloudType::Type &cloud_type) override;

        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> load_cloud(const std::string &cloud_name,
                                                                   const CloudType::Type &cloud_type) override;

        std::vector<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> load_clouds(
                const CloudType::Type &cloud_type) override;

        std::string get_scan_name();

        /**
         * Update the calibration .json file in the current scan folder with the
         * axis direction and center point.
         * @param dir axis direction.
         * @param pt center point.
         */
        void update_calibration_json(const equations::Normal &dir, const equations::Point &pt);

        void update_calibration_json(const equations::Normal &dir, const pcl::PointXYZ &pt);

    private:

        /**
         * Create a calibration .json file in the current calibration folder with 0 as default parameter.
         */
        void create_calibration_json();

        /**
         * Get the calibration file for the current calibration scan.
         * @return the calibration .json file.
         */
        nlohmann::json get_calibration_json();
    };
}

#endif //SWAG_SCANNER_CALIBRATIONFILEHANDLER_H
