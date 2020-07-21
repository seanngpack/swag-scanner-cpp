#include <iostream>
#include <thread>

#include <pcl/common/common_headers.h>
#include <algorithm>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

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
        void simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);


        /**
         * Visualize pointclouds. first cloud is white, then successive clouds are red and lighter shades of red.
         * @param clouds vector of pointcloud you want to visualize
         */
        void simpleVis(std::vector<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> clouds);

        /**
         *
         * @param cloud1
         * @param cloud2
         */
        void compareVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud1, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud2);

        /**
         * Visualize normal vectors.
         * @param cloud base cloud.
         * @param normals normal cloud for base cloud.
         */
        void normalsVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                                                          pcl::PointCloud<pcl::Normal>::ConstPtr normals);


    private:

    };
}

#endif //SWAG_SCANNER_VISUALIZER_H
