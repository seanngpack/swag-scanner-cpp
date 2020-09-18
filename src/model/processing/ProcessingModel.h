#ifndef SWAG_SCANNER_PROCESSINGMODEL_H
#define SWAG_SCANNER_PROCESSINGMODEL_H

#include "IModel.h"
#include "ScanFileHandler.h"

namespace model {
    class ProcessingModel : public IModel {
    public:
        ProcessingModel();

        ~ProcessingModel() = default;

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
         *  Remove NaN points, remove outliers.
         *
         * @param mean_k number of neighbors to analyze.
         * @param thresh_mult multipler for standard deviation, members outside st will be removed.
         */
        void filter(int mean_k = 50,
                    float thresh_mult = 1);

        /**
         * Transform clouds to world coordinate.
         * Loads the latest calibration for data used in transformation.
         * TODO: modify this so processModel can accept any calibration instead of just the latest.
         *
         * @throws runtime_error if the clouds vector is not loaded yet.
         */
        void transform_clouds_to_world();

        /**
         * Perform registration on clouds. Rotates them by scanning angle to original cloud.
         */
        void register_clouds();


    private:
        file::ScanFileHandler file_handler;

    };
}

#endif //SWAG_SCANNER_PROCESSINGMODEL_H
