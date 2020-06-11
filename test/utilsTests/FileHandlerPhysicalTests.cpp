// in this test suite we use the actual camera to gather depth data and test the file
// handling capabilities

#include <gtest/gtest.h>
#include "FileHandler.h"

using namespace testing;


class FileHandlerPhysicalFixture : public ::testing::Test {


protected:
    file::FileHandler *handler;
    std::string folder_path = "/Users/seanngpack/Programming Stuff/Projects/scanner_files/testing/FileHandlerPhysicalTests";

    virtual void SetUp() {
        handler = new file::FileHandler(false);
        set_up_test_files();
        handler->set_scan_folder_path(folder_path);
    }

    virtual void TearDown() {
        delete handler;
    }

    /**
    * If the folder doesn't exist then make it and save a pointcloud to it.
    * @param folder_path path to the testing folder setup.
    */
    void set_up_test_files() {
        if (!boost::filesystem::is_directory(folder_path)) {
            boost::filesystem::create_directory(folder_path);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = set_up_point_cloud();
            pcl::io::savePCDFileASCII(folder_path + "/" + "test_cloud.pcd", *cloud);
        }
    }

    static pcl::PointCloud<pcl::PointXYZ>::Ptr set_up_point_cloud() {
        // set up point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud->height = 10;
        cloud->width = 10;
        cloud->is_dense = true;
        cloud->points.resize(10 * 10);

        for (int y = 0; y < 10; y++) {
            for (int x = 0; x < 10; x++) {
                cloud->points[y * 10 + x] = pcl::PointXYZ(x, y, 1);
            }
        }
        return cloud;
    }
};


TEST_F(FileHandlerPhysicalFixture, TestLoadCloud) {
    // make an empty cloud to cloud the file into.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // load the file
    pcl::io::loadPCDFile<pcl::PointXYZ>(folder_path + "/" + "test_cloud.pcd", *cloud);
    ASSERT_EQ(cloud->height, 10);
    ASSERT_EQ(cloud->width, 10);
    ASSERT_EQ(cloud->size(), 100);
}

/**
 * Load clouds from a folder
 */
TEST_F(FileHandlerPhysicalFixture, TestLoadClouds) {
    // make an empty cloud to cloud the file into.
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr,
            Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ>::Ptr> > data;
    handler->load_clouds(data, CloudType::Type::RAW, folder_path);
    ASSERT_EQ(data.size(), 4);


}
