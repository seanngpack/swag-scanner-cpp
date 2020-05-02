#ifndef SWAG_SCANNER_FILEHANDLER_H
#define SWAG_SCANNER_FILEHANDLER_H

#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "CloudType.h"

namespace file {
    /**
     * Contains functions to handle file saving and loading.
     */
    class FileHandler {
    public:
        FileHandler();

        /**
         * Set the save file path.
         * @param path
         */
        void set_file_path(std::string path);

        /**
         * Save the given cloud to the file_path.
         * @param cloud the cloud you want to save.
         * @para cloud_type enum for the type of cloud you are saving.
         */
        void save_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, CloudType cloud_type);


        pcl::PointCloud<pcl::PointXYZ>::Ptr open_cloud(std::string cloud_name,
                CloudType cloud_type);

    private:
        std::string file_path;
    };
}
#endif //SWAG_SCANNER_FILEHANDLER_H
