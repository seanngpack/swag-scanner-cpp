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

namespace file {
    /**
     * Contains functions to handle file saving and loading.
     */
    class FileHandler {
    public:
        /**
         * Default file path argument is my data folder for now.
         */
        FileHandler(std::string folder_path = "/Users/seanngpack/Programming Stuff/Projects/scanner_files",
                bool auto_create_flag = false);

        /**
         * Set the scan_folder_path instance variable. Should be pointed to the scan folders.
         * E.g user/scanner_data/10
         * @param path the folder path.
         */
        void set_scan_folder_path(std::string path);

        /**
         * Get the current scan folder.
         * @return scan folder.
         */
        std::string get_scan_folder_path();

        /**
         * Save the given cloud to the current output_path.
         * @param cloud the cloud you want to save. This will change the folder
         * path that the cloud gets saved in.
         * @para cloud_type enum for the type of cloud you are saving.
         */
        void save_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                        std::string cloud_name,
                        CloudType cloud_type);

        /**
         * Load a pointcloud from the current_scan_folder given the name and type.
         * @param cloud_name name of the cloud.
         * @param cloud_type type of the cloud.
         * @return the cloud.
         */
        pcl::PointCloud<pcl::PointXYZ>::Ptr open_cloud(std::string cloud_name,
                                                       CloudType cloud_type);

    private:
        std::string all_data_folder_path;
        std::string scan_folder_path;

        const std::unordered_map<CloudType, std::string> type_path_map = {
                {CloudType::RAW,       "/raw"},
                {CloudType::FILTERED,  "/filtered"},
                {CloudType::SEGMENTED, "/segmented"},
                {CloudType::NORMAL,    "/normal"}
        };


        /**
         * Given the all data folder, find the current scan folder.
         * E.g. if there are scans 1->10 in the all data folder, that means the current
         * scan must be 11. Do not make the folder for the current scan yet.
         * @param folder the all data folder.
         *
         */
        std::string find_scan_folder(std::string folder);

        /**
         * Create the sub folders defined in CloudTypes in the scan_folder_path.
         */
        void create_sub_folders();

        /**
         * Check if the folder exists. If not, then throw an error.
         * @param folder the folder that houses all the scanner data.
         * @returns true if the input is good.
         */
        static bool check_input(std::string folder);
    };
}
#endif //SWAG_SCANNER_FILEHANDLER_H
