#ifndef SWAG_SCANNER_SCANMODEL_H
#define SWAG_SCANNER_SCANMODEL_H

#include "IModel.h"
#include "ScanFileHandler.h"

namespace pcl {
    class PointXYZ;

    template<class pointT>
    class PointCloud;
}

namespace model {
    /**
     * Represents a model for scanning.
     */
    class ScanModel : public IModel {
    public:
        ScanModel();

        ~ScanModel() = default;


        /**
         * Set the scan to the input. This triggers the filehandler to set the current working directory
         * to the given input. This will also clear any existing clouds in the model.
         *
         * @param scan_name scan name.
         */
        void set_scan(const std::string &scan_name);

        /**
         * Save scan cloud.
         */
        void save_cloud(const std::string &cloud_name, const CloudType::Type &cloud_type);

        /**
         * Save given cloud.
         */
        void save_cloud(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud,
                        const std::string &cloud_name,
                        const CloudType::Type &cloud_type);

        /**
         * Update the info.json file with the current time, number of degrees, and number of rotations.
         *
         * @param deg angle in degrees.
         * @num_rot number of rotations.
         */
        void update_info_json(int deg, int num_rot);

    private:
        std::shared_ptr<spdlog::logger> logger;
        file::ScanFileHandler file_handler;

    };
}

#endif //SWAG_SCANNER_SCANMODEL_H
