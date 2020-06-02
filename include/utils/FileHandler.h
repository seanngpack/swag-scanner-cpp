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
     * TODO: do some error handling to make sure you can disable
     * auto create folders but can still save files if you set the
     * folder manually.
     */
    class FileHandler {
    public:


        inline static const std::string default_data_path = "/Users/seanngpack/Programming Stuff/Projects/scanner_files";

        /**
         * Default file path argument is my data folder for now.
         * @param all_data_folder_path path to the folder containing all your pointcloud data.
         * @param auto_create_flag if true, create the folders at runtime, if false then don't.
         */
        explicit FileHandler(const std::string &all_data_folder_path = default_data_path,
                             bool auto_create_flag = true);

        /**
         * Overloaded constructor for only passing in a auto_create_flag.
         * Uses the default_data_path as the default path.
         * @param auto_create_flag if true, create the folders at runtime, if false then don't.
         */
        FileHandler(bool auto_create_flag);

        /**
         * Set the scan_folder_path instance variable. Should be pointed to the scan folders.
         * E.g user/scanner_data/10
         * Note: current behavior is it should create the new sub folders
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

        /**
         * NOTE: Currently does not support full paths.
         * Load a pointcloud from the current_scan_folder given the name and type.
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
         * Loads all clouds in the current scan folder into a vector.
         * @param cloud_vector the vector you want to load the clouds into.
         * @param cloud_type determines which folder to search for.
         */
        void load_clouds(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr,
                Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ>::Ptr> > &cloud_vector,
                         CloudType::Type cloud_type,
                         const std::string &folder_path = std::string());

    private:
        std::string all_data_folder_path;
        std::string scan_folder_path;
        bool auto_create_flag;


        /**
         * Given the all data folder, find the current scan folder.
         * E.g. if there are scans 1->10 in the all data folder, that means the current
         * scan must be 11. Do not make the folder for the current scan yet.
         * @param folder the all data folder.
         *
         */
        std::string find_scan_folder(const std::string &folder);

        /**
         * Create the sub folders defined in CloudTypes in the scan_folder_path.
         */
        void create_sub_folders();

        /**
         * Check if the folder exists. If not, then throw an error.
         * @param folder the folder that houses all the scanner data.
         * @returns true if the input is good.
         */
        static bool check_folder_input(const std::string &folder);

        /**
         * Check to see if a file exists given the path.
         * @param file_path the path to the file.
         * @return true if it exists.
         * @throws illegal argument exception if there isn't a file there.
         */
        static bool check_file_input(const std::string &file_path);
    };
}
#endif //SWAG_SCANNER_FILEHANDLER_H
