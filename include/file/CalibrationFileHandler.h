#ifndef SWAG_SCANNER_CALIBRATIONFILEHANDLER_H
#define SWAG_SCANNER_CALIBRATIONFILEHANDLER_H

#include "FileHandler.h"

namespace file {

    class CalibrationFileHandler : public FileHandler {
    public:

        // TODO: add constructors here

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
    };
}

#endif //SWAG_SCANNER_CALIBRATIONFILEHANDLER_H
