#include <thread>
#include <pcl/common/common_headers.h>
#include <memory>

#ifndef SWAG_SCANNER_VISUALIZER_H
#define SWAG_SCANNER_VISUALIZER_H

namespace visual {
    class Visualizer {
    public:

        Visualizer();


        /**
         * Visualize one pointcloud
         * @param cloud pointcloud you want to visualize
         */
        void simpleVis(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud);


        /**
         * Visualize pointclouds. first cloud is white, then successive clouds are red and lighter shades of red.
         * @param clouds vector of pointcloud you want to visualize
         */
        void simpleVis(const std::vector<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> &clouds);

        void simpleVisColor(const std::vector<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> &clouds);

        /**
         * Visualize a cloud and a point.
         * @param cloud the cloud.
         * @param pt the point.
         */
        void ptVis(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud, const pcl::PointXYZ &pt);

        /**
         * Visualize two point clouds side by side.
         * @param cloud1
         * @param cloud2
         */
        void compareVis(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud1,
                        const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud2);

        /**
         * Visualize normal vectors.
         * @param cloud base cloud.
         * @param normal normal cloud for base cloud.
         */
        void normalsVis(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud,
                        std::shared_ptr<pcl::PointCloud<pcl::Normal>> &normal);


    private:

    };
}

#endif //SWAG_SCANNER_VISUALIZER_H
