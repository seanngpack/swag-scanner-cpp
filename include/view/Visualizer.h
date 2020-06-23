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

        void simpleVis(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ>::Ptr>> clouds);

        /**
         * Visualize pointclouds. first cloud is white and then green to successive darker greens
         * @param clouds vector of pointcloud you want to visualize
         */
        void simpleVis(std::vector<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> clouds);

        void normalsVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                                                          pcl::PointCloud<pcl::Normal>::ConstPtr normals);


    private:

    };
}

#endif //SWAG_SCANNER_VISUALIZER_H
