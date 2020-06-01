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

        pcl::visualization::PCLVisualizer::Ptr simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);

        pcl::visualization::PCLVisualizer::Ptr normalsVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                                                          pcl::PointCloud<pcl::Normal>::ConstPtr normals);


    private:

    };
}

#endif //SWAG_SCANNER_VISUALIZER_H
