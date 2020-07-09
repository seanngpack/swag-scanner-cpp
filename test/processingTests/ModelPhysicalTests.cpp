/**
 * This is a "phsyical test" because it depends on the content in the /test folder.
 */
#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include "Model.h"
#include "Visualizer.h"
#include "Segmentation.h"
#include "Normal.h"
#include "Point.h"


class ModelPhysicalFixture : public ::testing::Test {

protected:
    model::Model *mod;
    std::string test_folder_path = "/Users/seanngpack/Programming Stuff/Projects/scanner_files/testing/FileHandlerPhysicalTests";

    virtual void SetUp() {
        mod = new model::Model();
    }

    virtual void TearDown() {
        delete mod;
    }
};


//TEST_F(ModelPhysicalFixture, TestICP) {
//    GTEST_SKIP();
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInFiltered(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutFiltered(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInSegmented(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutSegmented(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr finalCloud(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::io::loadPCDFile<pcl::PointXYZ>(test_folder_path + "/raw/" + "1.pcd", *cloudIn);
//    pcl::io::loadPCDFile<pcl::PointXYZ>(test_folder_path + "/raw/" + "2.pcd", *cloudOut);
//    std::cout << cloudIn->isOrganized() << std::endl;
//    mod->crop_cloud(cloudIn, -.15, .15,
//                    -100, .08,
//                    -100, .5);
//    std::cout << cloudIn->isOrganized() << std::endl;
//    mod->crop_cloud(cloudOut, -.15, .15,
//                    -100, .08,
//                    -100, .5);
//    std::vector<int> indices;
//    pcl::removeNaNFromPointCloud(*cloudIn, *cloudIn, indices);
//    pcl::removeNaNFromPointCloud(*cloudOut, *cloudOut, indices);
//
//    cloudInFiltered = mod->voxel_grid_filter(cloudIn, .0003);
//    cloudOutFiltered = mod->voxel_grid_filter(cloudOut, .0003);
//    mod->remove_plane(cloudInFiltered, cloudInSegmented);
//    mod->remove_plane(cloudOutFiltered, cloudOutSegmented);
//
//    pcl::io::savePCDFileASCII(test_folder_path + "/raw/inSeg.pcd", *cloudInSegmented);
//    pcl::io::savePCDFileASCII(test_folder_path + "/raw/outSeg.pcd", *cloudOutSegmented);
//
//
//    Eigen::Matrix4f transform = mod->icp_register_pair_clouds(cloudInSegmented, cloudOutSegmented, finalCloud);
//    std::cout << transform << std::endl;
//    pcl::transformPointCloud(*cloudInSegmented, *transformedCloud, transform);
//    std::vector<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> clouds{cloudOutSegmented, finalCloud};
//    visual::Visualizer visualizer;
//    visualizer.simpleVis(clouds);
////    visualizer.simpleVis(clouds);
//}


TEST_F(ModelPhysicalFixture, TestCalculateCenterPt) {
    equations::Normal axis_dir = equations::Normal(-0.0158, -0.8661, -0.4996);
    std::vector<equations::Plane> upright_planes = {
            equations::Plane(0.8603, -0.2446, 0.4472, -.201376),
            equations::Plane(0.7779, -0.3059, 0.5489, -.242753),
            equations::Plane(0.6714, -0.3638, 0.6457, -.28239),
            equations::Plane(-0.5484, -0.4098, 0.7289, -.316794),
            equations::Plane(-0.4123, -0.4443, 0.7954, -.344772),
            equations::Plane(-0.2635, -0.4709, 0.8419, -.364747),
            equations::Plane(-0.1081, -0.4863, 0.8670, -.376452),
            equations::Plane(-0.0492, -0.4900, 0.8703, -.379304),
            equations::Plane(-0.2061, -0.4751, 0.8555, -.374743),
            equations::Plane(-0.3655, -0.4537, 0.8128, -.359423)
    };

    equations::Point p = mod->calculate_center_pt(axis_dir, upright_planes);
    ASSERT_NEAR(p.x, -0.000213082, .001);
    ASSERT_NEAR(p.y, 0.0298714, .001);
    ASSERT_NEAR(p.z, 0.42673, .001);
}