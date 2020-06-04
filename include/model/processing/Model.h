/**
 * Model for processing point clouds. Holds a reference to the FileHandler for saving.
 * TODO: add field for save data location. Pass this data along to the FileHandler.
 */
#ifndef SWAG_SCANNER_MODEL_H
#define SWAG_SCANNER_MODEL_H

#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/registration/icp.h>
#include <FileHandler.h>
#include <CameraTypes.h>
#include <CloudType.h>

namespace model {

    /**
     * Procesing model that contains functions to manipulate depth frames and pointclouds.
     */
    class Model {

    public:
        /**
         * Constructor for Model.
         */
        explicit Model(bool auto_create_folders = true);

        /**
        * Create a new PointCloudXYZ using the instance variable depth_frame.
        * @return a boost pointer to the new pointcloud.
        */
        pcl::PointCloud<pcl::PointXYZ>::Ptr create_point_cloud(const uint16_t *depth_frame,
                                                               const camera::ss_intrinsics *intrinsics);

        /**
         * Take in a pointcloud, calculate the normals, and return a normal cloud.
         * Using integral images to compute normals much faster than standard plane fitting.
         * @return a normal cloud.
         */
        pcl::PointCloud<pcl::Normal>::Ptr estimate_normal_cloud(
                pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);

        /**
         * Applies crop box filtering to remove outside points from cloud.
         * @param cloud the cloud you want to crop.
         */
        void crop_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                        float minX, float maxX,
                        float minY, float maxY,
                        float minZ, float maxZ);

        pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_grid_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                                              float leafSize = .01);

        void remove_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut);

        /**
         * Use ICP to register an input and target cloud.
         * @returns a transformation matrix from the source to target cloud.
         */
        Eigen::Matrix4f register_pair_clouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn,
                                             pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut,
                                             pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud);

        /**
         * Save pointcloud to file.
         * @param cloud the cloud you want to save.
         */
        void save_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                        const std::string &name,
                        CloudType::Type cloud_type);


        /**
         * Calls on filehandler's load cloud method. Refer to that for full
         * documentation.
         * TODO: Check the pass by value warning.
         * TODO: Make this function like load_clouds where the model method
         * can take in a path.
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
         * Load the clouds from a folder into a vector of clouds. Will throw error
         * if auto_create_folder is false AND you try to load clouds without specifying a path.
         * @param cloud_vector vector storing the loaded clouds.
         * @param cloud_type tells function which sub folder to look for.
         * @param folder_path path to the main scan folder.
         */
        void load_clouds(
                std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ>::Ptr>> &cloud_vector,
                CloudType::Type cloud_type,
                const std::string &folder_path = std::string());

        ~Model();

    private:
        bool auto_create_folders;
        file::FileHandler fileHandler;
    };
}

#endif //SWAG_SCANNER_MODEL_H
