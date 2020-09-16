#include <thread>
#include <pcl/common/common_headers.h>
#include <memory>

#ifndef SWAG_SCANNER_VISUALIZER_H
#define SWAG_SCANNER_VISUALIZER_H

namespace pcl {
    namespace visualization {
        class PointPickingEvent;
    }
}

namespace visual {
    class Visualizer {
    public:

        Visualizer();

        static void pointPickingEventOccurred(const pcl::visualization::PointPickingEvent& event, void* viewer_void);

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
         * Visualize four points clouds in a grid.
         * @param cloud1
         * @param cloud2
         * @param cloud3
         * @param cloud4
         */
        void compareVisFour(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud1,
                            const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud2,
                            const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud3,
                            const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud4,
                            const std::string &cloud1_desc = "cloud 1",
                            const std::string &cloud2_desc = "cloud 2",
                            const std::string &cloud3_desc = "cloud 3",
                            const std::string &cloud4_desc = "cloud 4");

        /**
         * Visualize normal vectors.
         * @param cloud base cloud.
         * @param normal normal cloud for base cloud.
         */
        void normalsVis(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud,
                        const std::shared_ptr<pcl::PointCloud<pcl::Normal>> &normal);


    private:

    };
}

#endif //SWAG_SCANNER_VISUALIZER_H
