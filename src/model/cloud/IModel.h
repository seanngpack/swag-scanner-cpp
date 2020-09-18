#ifndef SWAG_SCANNER_IMODEL_H
#define SWAG_SCANNER_IMODEL_H

#include <memory>
#include <vector>
#include <map>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>

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
         * @param cloud cloud to add.
         * @param cloud_name name of cloud.
         */
        inline void add_cloud(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud, const std::string &cloud_name) {
            clouds.push_back(cloud);
            clouds_map.insert({cloud_name, clouds.size() - 1});
        }

        /**
         * Applies crop box filtering to remove outside points from cloud in place.
         *
         * @param cloud the cloud you want to crop.
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
        }

        /**
        * Downsample the given cloud using voxel grid in place.
         *
        * @param cloud cloud you want to downsample.
        * @param leafSize size of leaf.
        */
        inline void voxel_grid_filter(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud,
                                      float leafSize = .01) {
            pcl::VoxelGrid<pcl::PointXYZ> grid;
            std::cout << "PointCloud before filtering: " << cloud->width * cloud->height
                      << " data points (" << pcl::getFieldsList(*cloud) << ")." << std::endl;
            grid.setInputCloud(cloud);
            grid.setLeafSize(leafSize, leafSize, leafSize);
            grid.filter(*cloud);
            std::cout << "PointCloud after filtering: " << cloud->width * cloud->height
                      << " data points (" << pcl::getFieldsList(*cloud) << ")." << std::endl;
        }

        /**
         * Clear stored pointclouds from memory.
         */
        inline void clear_clouds() {
            clouds.clear();
            clouds_map.clear();
        }

        virtual ~

        IModel() {}

    protected:
        std::vector<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> clouds;
        std::map<std::string, int> clouds_map;

    };
}

#endif //SWAG_SCANNER_IMODEL_H
