#include "ProcessingModel.h"
#include "Visualizer.h"
#include "Algorithms.h"
#include "Normal.h"
#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <filesystem>

namespace fs = std::filesystem;

class TwoDHoleFillingFixture : public ::testing::Test {

protected:
    model::ProcessingModel *mod;
    visual::Visualizer *viewer;

    virtual void SetUp() {
        mod = new model::ProcessingModel();
        viewer = new visual::Visualizer();
    }

    virtual void TearDown() {
        delete mod;
        delete viewer;
    }
};

TEST_F(TwoDHoleFillingFixture, LetsDoIt) {
    auto rabbit_registered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::io::loadPCDFile<pcl::PointXYZ>(fs::current_path().string() + "/research/meshing/data/rabbit_registered.pcd",
                                        *rabbit_registered);

    // STEP1: get all points within a given distance from ground plane.
    //
    //
    float dist = .009; // 9mm from the ground plane
    auto inliers = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();


    // this loop is for unorganized clouds. For organized clouds, need double for
    for (int i = 0; i < rabbit_registered->size(); i++) {
        auto pt = rabbit_registered->points[i];
        if (pt.z < dist) {
            inliers->push_back(pt);
        }
    }

    std::cout << "inliers size: " << inliers->size() << std::endl;

//    mod->remove_outliers(inliers, 50, 1);
//    mod->remove_outliers_radius(inliers, .001, 1000);

    visual::Visualizer::simpleVis(inliers);

    auto cloud_2d = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    // project inliers to ground plane
    for (int i = 0; i < inliers->size(); i++) {
        auto projected = algos::project_point_to_plane(inliers->points[i], pcl::PointXYZ(0, 0, 0),
                                                       equations::Normal(0, 0, 1));
        cloud_2d->push_back(projected);
    }

//    mod->remove_outliers(cloud_2d, 50, 1);
    mod->remove_outliers_radius(cloud_2d, .005, 100);
    visual::Visualizer::simpleVis(cloud_2d);

    std::cout << "cloud_2d size: " << cloud_2d->size() << std::endl;


    // perform concave hull
    //
    //
//    auto cloud_hull = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
//    pcl::ConcaveHull<pcl::PointXYZ> chull;
//    chull.setInputCloud(cloud_2d);
//    chull.setAlpha(0.001);
//    chull.reconstruct(*cloud_hull);
//
//    visual::Visualizer::simpleVis(cloud_hull);

//    visual::Visualizer::simpleVis(cloud_2d);

    // let's see what convex hull would look like
    auto convex_hull = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::ConvexHull<pcl::PointXYZ> convhull;
    convhull.setInputCloud(cloud_2d);
    convhull.setDimension(2);
//    convhull.setAlpha(0.0005);
    convhull.reconstruct(*convex_hull);

    visual::Visualizer::simpleVis(convex_hull);

    std::cout << convex_hull->size() << std::endl;

}
