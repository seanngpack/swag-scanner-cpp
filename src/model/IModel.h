#ifndef SWAG_SCANNER_IMODEL_H
#define SWAG_SCANNER_IMODEL_H

#include "CloudType.h"
#include "Logger.h"
#include <memory>
#include <vector>
#include <map>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>

namespace pcl {
    class PointXYZ;

    template<class pointT>
    class PointCloud;
}


namespace model {
    /**
     * Interface for scanning, calibration, and processing models.
     */
    class IModel {
    public:
        IModel() = default;

        /**
         * Add clouds to vector and keep a mapping of its position with a mpa of its name and index.
         *
         * @param cloud calibration to add.
         * @param cloud_name name of calibration.
         */
        inline void add_cloud(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud, const std::string &cloud_name) {
            clouds.push_back(cloud);
            clouds_map.insert({cloud_name, clouds.size() - 1});
        }

        /**
         * Return shared pointer to the calibration given its name.
         *
         * @param cloud_name name of calibration you want to get.
         * @return the calibration.
         * @throws runtime error if the name does not exist in the map.
         */
        inline std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> get_cloud(const std::string &cloud_name) {
            int index;
            if (clouds_map.find(cloud_name) == clouds_map.end()) {
                throw std::runtime_error("Error, calibration with name" + cloud_name + "does not exist");
            } else {
                index = clouds_map[cloud_name];
                return clouds[index];
            }
        }

        /**
         * Calculate and return the normals for input cloud.
         *
         * @param cloud cloud you want to calculate normals for.
         * @return the normals.
         */
        inline std::shared_ptr<pcl::PointCloud<pcl::Normal>> calculate_normals(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud) {
            pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
            auto normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();
            auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
            tree->setInputCloud(cloud);
            n.setInputCloud(cloud);
            n.setSearchMethod(tree);
            n.setKSearch(20);
            n.compute(*normals);
            return normals;
        }

        /**
         * Applies crop box filtering to remove outside points from calibration in place.
         * Keeps track of removed indices, slightly slower but necessary for logging.
         *
         * @param cloud the calibration you want to crop.
         */
        inline void crop_cloud(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud,
                               float minX, float maxX,
                               float minY, float maxY,
                               float minZ, float maxZ) {
            pcl::CropBox<pcl::PointXYZ> boxFilter;
            boxFilter.setKeepOrganized(1);
            boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
            boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
            boxFilter.setInputCloud(cloud);
            boxFilter.filter(*cloud);
            auto removed_indices = boxFilter.getRemovedIndices();
            logger::info("applied box filter, removed " + std::to_string(removed_indices->size()) + " points");
        }

        /**
         * Crop box and return a copy, does not affect input cloud.
         * Keeps track of removed indices, slightly slower but necessary for logging.
         *
         */
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>
        crop_cloud_cpy(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud,
                       float minX, float maxX,
                       float minY, float maxY,
                       float minZ, float maxZ) {
            auto cropped = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
            pcl::CropBox<pcl::PointXYZ> boxFilter(true);
            boxFilter.setKeepOrganized(1);
            boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
            boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
            boxFilter.setInputCloud(cloud);
            boxFilter.filter(*cropped);
            auto removed_indices = boxFilter.getRemovedIndices();
            logger::info("applied box filter, removed " + std::to_string(removed_indices->size()) + " points");
            return cropped;
        }

        /**
        * Downsample the given calibration using voxel grid in place.
         *
        * @param cloud calibration you want to downsample.
        * @param leafSize size of leaf.
        */
        inline void voxel_grid_filter(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud,
                                      float leafSize = .01) {
            pcl::VoxelGrid<pcl::PointXYZ> grid;
            int cloud_size_before = cloud->width * cloud->height;
            grid.setInputCloud(cloud);
            grid.setLeafSize(leafSize, leafSize, leafSize);
            grid.filter(*cloud);
            int cloud_size_after = cloud->width * cloud->height;
            logger::info("applied voxel grid downsampling (leafSize=" + std::to_string(leafSize) + ")" +
                         " point cloud size before: " + std::to_string(cloud_size_before) + ", after: " +
                         std::to_string(cloud_size_after));

        }

        /**
         * Fast Bilateral filtering for cloud in place.
         * //Note: when I move on to XYZRGB, use just the bilateral filter
         *
         * @param cloud cloud to filter.
         * @param sigma_s size of Gaussian bilateral filter window.
         * @param sigma_r  the standard deviation of the Gaussian used to control how much an
         * adjacent pixel is downweighted because of the intensity difference (depth in our case).
         */
        inline void bilateral_filter(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud,
                                     float sigma_s = 5,
                                     float sigma_r = 5e-3) {
            pcl::FastBilateralFilter<pcl::PointXYZ> bilateral;
            bilateral.setInputCloud(cloud);
            bilateral.setSigmaS(sigma_s);
            bilateral.setSigmaR(sigma_r);
            bilateral.applyFilter(*cloud);
            logger::info("applied bilateral filter (sigma_s=" + std::to_string(sigma_s) +
                         ", sigma_r=" + std::to_string(sigma_r) + ")");
        }


        /**
         * Remove outliers from calibration in place. Keep calibration organized.
         *
         * @param cloud calibration to filter.
         * @param mean_k number of neighbors to analyze.
         * @param thresh_mult multipler for standard deviation, members outside st will be removed.
         * @return filtered calibration.
         */
        inline void remove_outliers(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud,
                                    int mean_k = 50,
                                    float thresh_mult = 1) {
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
            sor.setInputCloud(cloud);
            sor.setMeanK(mean_k);
            sor.setStddevMulThresh(thresh_mult);
            sor.setKeepOrganized(true);
            sor.filter(*cloud);

            auto removed_indices = sor.getRemovedIndices();
            logger::info("applied outlier removal (mean_k= " + std::to_string(mean_k) + ", thresh_mult=" +
            std::to_string(thresh_mult) + ") removed " + std::to_string(removed_indices->size()) + " outliers");
        }

        /**
         * Remove NaN points from calibration in place. Organized clouds become unorganized from this.
         *
         * @param cloud calibration to remove points from.
         * @return
         */
        inline void remove_nan(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud) {
            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
            logger::info("applied NaN filter, removed " + std::to_string(indices.size()) + " NaN points");
        }

        /**
         * Clear stored pointclouds from memory.
         */
        inline void clear_clouds() {
            clouds.clear();
            clouds_map.clear();
        }

        virtual ~IModel() {}

    protected:
        std::vector<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> clouds;
        std::map<std::string, int> clouds_map;
    };
}

#endif //SWAG_SCANNER_IMODEL_H
