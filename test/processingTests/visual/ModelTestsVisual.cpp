/**
 * This is a "phsyical test" because it depends on the content in the /test folder.
 */
#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
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


TEST_F(ModelPhysicalFixture, TestICP) {
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloudIn(new pcl::PointCloud<pcl::PointXYZ>);
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloudOut(new pcl::PointCloud<pcl::PointXYZ>);
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloudInFiltered(new pcl::PointCloud<pcl::PointXYZ>);
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloudOutFiltered(new pcl::PointCloud<pcl::PointXYZ>);
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloudInSegmented(new pcl::PointCloud<pcl::PointXYZ>);
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloudOutSegmented(new pcl::PointCloud<pcl::PointXYZ>);
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> finalCloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> transformedCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>(test_folder_path + "/raw/" + "1.pcd", *cloudIn);
    pcl::io::loadPCDFile<pcl::PointXYZ>(test_folder_path + "/raw/" + "2.pcd", *cloudOut);
    std::cout << cloudIn->isOrganized() << std::endl;
    mod->crop_cloud(cloudIn, -.15, .15,
                    -100, .08,
                    -100, .5);
    std::cout << cloudIn->isOrganized() << std::endl;
    mod->crop_cloud(cloudOut, -.15, .15,
                    -100, .08,
                    -100, .5);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloudIn, *cloudIn, indices);
    pcl::removeNaNFromPointCloud(*cloudOut, *cloudOut, indices);

    cloudInFiltered = mod->voxel_grid_filter(cloudIn, .0003);
    cloudOutFiltered = mod->voxel_grid_filter(cloudOut, .0003);
    cloudInSegmented = mod->remove_plane(cloudInFiltered);
    cloudOutSegmented = mod->remove_plane(cloudOutFiltered);

    pcl::io::savePCDFileASCII(test_folder_path + "/raw/inSeg.pcd", *cloudInSegmented);
    pcl::io::savePCDFileASCII(test_folder_path + "/raw/outSeg.pcd", *cloudOutSegmented);


    Eigen::Matrix4f transform = mod->icp_register_pair_clouds(cloudInSegmented, cloudOutSegmented, finalCloud);
    std::cout << transform << std::endl;
    pcl::transformPointCloud(*cloudInSegmented, *transformedCloud, transform);
    std::vector<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> clouds{cloudOutSegmented, finalCloud};
    visual::Visualizer visualizer;
    visualizer.simpleVis(clouds);
//    visualizer.simpleVis(clouds);
}


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