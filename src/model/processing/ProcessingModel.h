#ifndef SWAG_SCANNER_PROCESSINGMODEL_H
#define SWAG_SCANNER_PROCESSINGMODEL_H

#include "IModel.h"
#include "ScanFileHandler.h"

namespace spdlog {
    class logger;
}

namespace model {
    class ProcessingModel : public IModel {
    public:
        ProcessingModel();

        ~ProcessingModel() = default;

        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> load_cloud(const std::string &name,
                                                                   const CloudType::Type type);

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
         * Use ICP registration between two clouds.
         *
         * @param cloud_src cloud source.
         * @param cloud_target cloud target.
         * @param transformed_cloud copy of source -> target.
         * @return matrix transformation of source -> target.
         */
        Eigen::Matrix4f icp_register_pair_clouds(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud_src,
                                                 const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud_target,
                                                 std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &transformed_cloud);

        /**
         * Do cropping, Run bilateral filter, Remove NaN points, remove outliers.
         *
         * @param sigma_s filter window for bilateral filter.
         * @param sigma_r standard deviation of the gaussian for bilateral filter.
         * @param mean_k number of neighbors to analyze.
         * @param thresh_mult multipler for standard deviation, members outside st will be removed.
         */
        void filter(int sigma_s = 10,
                    float sigma_r = .01,
                    int mean_k = 50,
                    float thresh_mult = 1);


        /**
         * Mesh registered cloud. Will explode if registered folder does not exist with cloud inside.
         */
        void mesh();

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
        std::shared_ptr<spdlog::logger> logger;
        file::ScanFileHandler file_handler;

    };
}

#endif //SWAG_SCANNER_PROCESSINGMODEL_H
