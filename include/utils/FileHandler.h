#ifndef SWAG_SCANNER_FILEHANDLER_H
#define SWAG_SCANNER_FILEHANDLER_H

#include <boost/filesystem.hpp>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "CloudType.h"
#include <string>
#include <unordered_map>
#include <CoreServices/CoreServices.h>


namespace file {
    /**
     * Contains functions to handle file saving and loading.
     */
    class FileHandler {
    public:

        /**
         * Sorting function that sorts files and directories numerically in order from lowest to greatest.
         * @param path1 first path.
         * @param path2 second path.
         * @return true if the first file is smaller than the second, false otherwise.
         */
        inline static bool path_sort(boost::filesystem::path &path1, boost::filesystem::path &path2) {
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

        /**
         * Default constructor makes a FileHandler. Searches for SwagScanner in the /applications path
         * and creates a new SwagScanner directory if it doesn't exist. Will also create a folder "1" under
         * the /data directory and set it as the current scan folder. * Then it will update the settings.json on the
         * latest scan. If SwagScanner exists, then it will use the current scan folder according to the
         * settings.json file.
         */
        FileHandler();

        /**
         * Constructor takes in a flag and determines whether to create a new scan folder
         * or not. If the flag is set to true then it will create a new scan folder numerically.
         * Then it will update the settings.json on the latest scan.
         * @param auto_create_flag
         */
        FileHandler(bool auto_create_flag);

        /**
         * Crates a scan folder with the given scan name. If the scan is already there,
         * then just set the current scan folder to it and do not touch anything inside the directory.
         *
         * @param scan_name name of the scan.
         */
        FileHandler(const char *scan_name);


        /**
         * Set the scan_folder_path instance variable. Should be pointed to the scan folders.
         * E.g user/scanner_data/10
         * If the subfolders don't exist then create them.
         * @param path the folder path.
         */
        void set_scan_folder_path(const std::string &path);

        /**
         * Get the current scan folder.
         * @return scan folder.
         */
        std::string get_scan_folder_path();

        /**
         * NOTE: Currently does not support full paths.
         * Save the given cloud to the current output_path.
         * @param cloud the cloud you want to save. This will change the folder
         * path that the cloud gets saved in.
         * @para cloud_type enum for the type of cloud you are saving.
         */
        void save_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                        const std::string &cloud_name,
                        CloudType::Type cloud_type);

        /*
         * Load a pointcloud from the scan folder given the name and type.
         * @param cloud the cloud you want to load the cloud into.
         * @param cloud_name name of the cloud.
         * @param cloud_type type of the cloud.
         *
         * Example: load_cloud("12.pcd", CloudType::RAW)
         */
        void load_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                        const std::string &cloud_name,
                        CloudType::Type cloud_type);


        /**
         * Loads all clouds in the current scan folder into a vector given the cloud type.
         * Only works with files that have numbers in the file name.
         * @param cloud_vector the vector you want to load the clouds into.
         * @param cloud_type determines which folder to search for.
         */
        void load_clouds(
                std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr,
                        Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ>::Ptr> > &cloud_vector,
                CloudType::Type cloud_type);

    private:
        std::string swag_scanner_path = []() {
            FSRef ref;
            OSType folderType = kApplicationSupportFolderType;
            char path[PATH_MAX];

            FSFindFolder(kUserDomain, folderType, kCreateFolder, &ref);
            FSRefMakePath(&ref, (UInt8 *) &path, PATH_MAX);
            std::string program_folder = "/SwagScanner";
            program_folder = path + program_folder;
            return program_folder;
        }();
        std::string scan_folder_path;


        /**
         * Go to the SwagScanner/calibration directory and find the latest calibration by date.
         * @return path to the latest calibration.
         */
        std::string find_latest_calibration();

        /**
         * Finds the last scan folder using the settings.json file in the /settings directory.
         * @return path to the latest scan.
         */
        std::string find_latest_scan_folder();

        /**
         * Find the current scan folder by sorting the existing scans numerically.
         * E.g. if there are scans 1->10 in the all data folder, that means the current
         * scan must be 11.
         * Does not make the folder for the current scan.
         * @param folder the all data folder.
         *
         */
        std::string find_latest_scan_folder_numeric(const std::string &folder);

        /**
         * Checks to see if a /SwagScanner folder exists in Library/Application Support.
         * If the folder does not exist, then create one. Otherwise, continue.
         * @returns true if the program folder is already there. False if it isn't.
         */
        bool check_program_folder();

        /**
         * Create the sub folders defined in CloudTypes in the scan_folder_path if they
         * don't exist.
         * Also creates a /calibration folder with a calibration_info.txt file
         */
        void create_sub_folders();

        /**
         * Update the settings.json file "latest_scan" field.
         */
        void set_settings_latest_scan(std::string &folder_path);

        /**
         * Update the json file in the latest scan with the given info.
         * @param date current date and time.
         * @param angle angle intervals of the scan.
         * @param cal calibration path.
         */
        void update_info_json(std::string date, std::string angle, std::string cal);

        /**
         * Check if the folder exists.
         * @param folder the folder path.
         * @returns true if the folder exists, false otherwise.
         */
        bool check_folder_input(const std::string &folder);

    };
}
#endif //SWAG_SCANNER_FILEHANDLER_H
