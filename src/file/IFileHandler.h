#ifndef SWAG_SCANNER_IFILEHANDLER_H
#define SWAG_SCANNER_IFILEHANDLER_H

#include <boost/filesystem.hpp>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "../types/CloudType.h"
#include <string>
#include <unordered_map>
#include <CoreServices/CoreServices.h>
#include <nlohmann/json.hpp>

namespace file {
    /**
     * Abstract class for File Handling objects.
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
        static void write_settings_json(nlohmann::json j);

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
        virtual void save_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                const std::string &cloud_name,
                                CloudType::Type cloud_type) = 0;

        /*
         * Load a pointcloud from the scan folder given the name and type.
         * @param cloud the cloud you want to load the cloud into
         * @param cloud_name name of the cloud.
         * @param cloud_type type of the cloud.
         *
         * Example: load_cloud(cloud, "testScan", "12.pcd", CloudType::RAW)
         */
        virtual void load_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                const std::string &cloud_name,
                                CloudType::Type cloud_type) = 0;


        /**
         * Loads all clouds in the current scan folder into a vector given the cloud type.
         * Only works with files that have numbers in the file name.
         *
         * @Edge case when passed CALIBRATION as the type, it will search through the
         * /calibration folder in the root directory and load the latest calibration.
         * @param cloud_vector the vector you want to load the clouds into.
         * @param cloud_type determines which folder to search for.
         *
         */
        virtual void load_clouds(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr,
                Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ>::Ptr> > &cloud_vector,
                                 CloudType::Type cloud_type) = 0;


        virtual std::string get_scan_name() = 0;

        virtual void set_scan_name(const std::string &scan_name) = 0;

        virtual ~IFileHandler() {
            std::cout << "calling IFileHandler destructor \n";
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
        static bool path_sort(boost::filesystem::path &path1, boost::filesystem::path &path2);

        /**
         * Find the next scan folder by sorting the existing scans numerically.
         * E.g. if there are scans 1->10 in the all data folder, that means the next
         * scan must be 11.
         *
         */
        virtual boost::filesystem::path
        find_next_scan_folder_numeric(CloudType::Type const &type = CloudType::Type::NONE);
    };
}
#endif //SWAG_SCANNER_IFILEHANDLER_H
