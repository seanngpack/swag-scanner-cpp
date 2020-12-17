#include "Visualizer.h"
#include <gtest/gtest.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
#include <filesystem>

namespace fs = std::filesystem;

TEST(wetesting, wetaeting) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>),
            cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>),
            cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);

//    pcl::PCDReader reader;
//    reader.read(fs::current_path().string() + "/research/meshing/data/table_scene_mug_stereo_textured.pcd", *cloud);
    pcl::PCDReader reader;
    reader.read(fs::current_path().string() + "/research/meshing/data/rabbit_registered.pcd", *cloud);

    visual::Visualizer::simpleVis(cloud);
    // Build a filter to remove spurious NaNs
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0, 1.1);
    pass.filter(*cloud_filtered);
    std::cerr << "PointCloud after filtering has: "
              << cloud_filtered->size() << " data points." << std::endl;

    visual::Visualizer::simpleVis(cloud_filtered);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    // Create the segmentation object
//    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
//    seg.setOptimizeCoefficients(true);
//    // Mandatory
//    seg.setModelType(pcl::SACMODEL_PLANE);
//    seg.setMethodType(pcl::SAC_RANSAC);
//    seg.setDistanceThreshold(0.01);
//
//    seg.setInputCloud(cloud_filtered);
//    seg.segment(*inliers, *coefficients);
//    std::cerr << "PointCloud after segmentation has: "
//              << inliers->indices.size() << " inliers." << std::endl;

    std::cout << "seems like i cant manually assign values" << std::endl;
    coefficients->values.resize(4);
    coefficients->values[0] = 0.0f;
    coefficients->values[1] = 0.0f;
    coefficients->values[2] = 1.0f;
    coefficients->values[3] = 0.0f;

    std::cout << "never makes it here" << std::endl;

    std::cerr << "Model coefficients: " << coefficients->values[0] << " "
              << coefficients->values[1] << " "
              << coefficients->values[2] << " "
              << coefficients->values[3] << std::endl;

    // Project the model inliers
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    // proj.setIndices (inliers);
    proj.setInputCloud(cloud_filtered);
    proj.setModelCoefficients(coefficients);
    proj.filter(*cloud_projected);
    std::cerr << "PointCloud after projection has: "
              << cloud_projected->size() << " data points." << std::endl;

    visual::Visualizer::simpleVis(cloud_projected);

    // Create a Concave Hull representation of the projected inliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConcaveHull<pcl::PointXYZ> chull;
    chull.setInputCloud(cloud_projected);
    chull.setAlpha(0.01);
    chull.reconstruct(*cloud_hull);

    std::cerr << "Concave hull has: " << cloud_hull->size()
              << " data points." << std::endl;

    visual::Visualizer::simpleVis(cloud_hull);
    pcl::PCDWriter writer;
    writer.write("table_scene_mug_stereo_textured_hull.pcd", *cloud_hull, false);

}