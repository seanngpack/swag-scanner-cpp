#ifndef SWAG_SCANNER_IFILEHANDLER_H
#define SWAG_SCANNER_IFILEHANDLER_H

#include <boost/filesystem.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "CloudType.h"
#include <string>
#include <unordered_map>
#include <nlohmann/json.hpp>

namespace file {
    /**
     * Abstract base class for File Handling objects.
     * This class is specialized into CalibrationFileHandler and ScanFileHandler, so it serves more as
     * an interface. There is currently no need to modify this class to be a base type for CalibrationFileHandler
     * and ScanFileHandler.
     */
    class IFileHandler {
    public:

        /**
         * Static method get the settings.json file from root of project.
         * @return json file.
         */
        static nlohmann::json load_settings_json();

        /**
         * Static method write to settings.json.
         * @param j json file that follows format of settings.json
         */
        static void write_settings_json(const nlohmann::json &j);

        /**
         * Go to the SwagScanner/calibration directory and find the latest calibration by date.
         * @return path to the latest calibration.
         *
         * ex: find_latest_calibration() -> .../SwagScanner/calibration/testCal1/testCal1.json
         */
        virtual boost::filesystem::path find_latest_calibration();

        /**
         * Save the given cloud to the current output_path.
         * @param cloud the cloud you want to save.
         * @para cloud_type enum for the type of cloud you are saving. Affects the subfolder path.
         */
        virtual void save_cloud(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud,
                                const std::string &cloud_name,
                                const CloudType::Type &cloud_type) = 0;

        /*
         * Load a pointcloud from the scan folder given the name and type.
         *
         * @param cloud_name name of the cloud.
         * @param cloud_type type of the cloud.
         * @return cloud.
         *
         * Example: load_cloud("12", CloudType::RAW)
         */
        virtual std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> load_cloud(const std::string &cloud_name,
                                                                           const CloudType::Type &cloud_type) = 0;

        /**
         * Get all the scans.
         *
         * @return vector of all the scan names.
         */
        static std::vector<std::string> get_all_scans();

        /**
         * Get all the calibrations.
         *
         * @return vector of all the calibration names.
         */
        static std::vector < std::string > get_all_calibrations();


        /**
         * Loads all clouds in the current scan folder into a vector given the cloud type.
         * Only works with files that have numbers in the file name.
         *
         * @Edge case when passed CALIBRATION as the type, it will search through the
         * /calibration folder in the root directory and load the latest calibration.
         *
         * @param cloud_type determines which folder to search for.
         * @return vector of loaded cloud pointers.
         *
         */
        virtual std::vector<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>>
        load_clouds(const CloudType::Type &cloud_type) = 0;


        virtual ~IFileHandler() {
            std::cout << "IFilehandler destructor" << std::endl;
        }

    protected:
        static boost::filesystem::path swag_scanner_path;
        boost::filesystem::path scan_folder_path;
        std::string scan_name;

        /**
         * Sorting function that sorts files and directories numerically in order from lowest to greatest.
         * @param path1 first path.
         * @param path2 second path.
         * @return true if the first file is smaller than the second, false otherwise.
         */
        static bool path_sort(const boost::filesystem::path &path1, const boost::filesystem::path &path2);

        /**
         * Find the next scan folder by sorting the existing scans numerically.
         * E.g. if there are scans 1->10 in the all data folder, that means the next
         * scan must be 11.
         *
         */
        virtual boost::filesystem::path
        find_next_scan_folder_numeric(const CloudType::Type &type);

        boost::filesystem::path find_next_scan_folder_numeric();
    };
}
#endif //SWAG_SCANNER_IFILEHANDLER_H
