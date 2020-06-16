#ifndef SWAG_SCANNER_PROCESSINGCONTROLLER_H
#define SWAG_SCANNER_PROCESSINGCONTROLLER_H

#include "Model.h"
#include "FileHandler.h"
#include "Visualizer.h"

/**
 * This controller handles data processing commands.
 */
namespace controller {
    class ProcessingController {
    public:
        ProcessingController(std::shared_ptr<model::Model> model,
                             visual::Visualizer *viewer,
                             std::shared_ptr<file::FileHandler> file_handler);

        /**
        * Process the data. Currently will process the most recently scanned.
        */
        void process_data();

        /**
         * Crop, downsample, and save the clouds to the /filtered folder
         * @param folder_path path to the scan folder.
         * @param which cloud types do you want to crop & filter. You should probably do
         * the /raw folder.
         * @param leaf size.
         */
        void filter_clouds(std::string folder_path, CloudType::Type cloud_type, float leaf_size);

        /**
         * Segment and save the clouds in the given folder path to the /segmented folder.
         * @param folder_path path to the scans.
         * @param cloud_type type of cloud you want to filter. You should probably do the
         * /filtered folder.
         */
        void segment_clouds(std::string folder_path, CloudType::Type cloud_type);

        /**
         * Register all point clouds in given folder location.
         * @param folder_path the path to the scan folder.
         * @param cloud_type the type of the cloud, tells which folder to look into for clouds.
         */
        void register_all_clouds(std::string folder_path, CloudType::Type cloud_type);

        /**
         * Register all point clouds in the current working folder. Default folder to register
         * is in the /raw folder.
         * TODO: Change the default folder from /raw to something else.
         */
        void register_all_clouds();

        void visualize_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

        ~ProcessingController();

    private:
        std::shared_ptr<model::Model> model;
        visual::Visualizer *viewer;
        std::shared_ptr<file::FileHandler> file_handler;
    };


}

#endif //SWAG_SCANNER_PROCESSINGCONTROLLER_H