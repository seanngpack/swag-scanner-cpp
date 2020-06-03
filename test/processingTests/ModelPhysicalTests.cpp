#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include "Model.h"
#include "Vizualizer.h"


class ModelPhysicalFixture : public ::testing::Test {

protected:
    model::Model *mod;
    std::string test_folder_path = "/Users/seanngpack/Programming Stuff/Projects/scanner_files/testing/FileHandlerPhysicalTests";

    virtual void SetUp() {
        mod = new model::Model(false);
    }

    virtual void TearDown() {
        delete mod;
    }
};


TEST_F(ModelPhysicalFixture, TestLoadCloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInFiltered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutFiltered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>(test_folder_path + "/raw/" + "1.pcd", *cloudIn);
    pcl::io::loadPCDFile<pcl::PointXYZ>(test_folder_path + "/raw/" + "2.pcd", *cloudOut);
    mod->crop_cloud(cloudIn, -.15, .15,
                    -100, .08,
                    -100, .5);
    mod->crop_cloud(cloudOut, -.15, .15,
                    -100, .08,
                    -100, .5);
    cloudInFiltered = mod->voxel_grid_filter(cloudIn);
    cloudOutFiltered = mod->voxel_grid_filter(cloudOut);
    visual::Visualizer visualizer;



    mod->register_pair_clouds(cloudIn, cloudOut);
    visualizer.simpleVis()
}