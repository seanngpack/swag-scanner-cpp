//// in this test suite we use the actual camera to gather depth data and test the file
//// handling capabilities
//
//#include <gtest/gtest.h>
//#include "ScanFileHandler.h"
//
//using namespace testing;
//
//
//class ScanFileHandlerPhysicalFixture : public ::testing::Test {
//
//
//protected:
//    file::ScanFileHandler *handler;
//    std::string folder_path = "/Users/seanngpack/Library/Application Support/SwagScanner/scans/scan_handler_test/raw";
//    virtual void SetUp() {
//        handler = new file::ScanFileHandler("scan_handler_test");
//        set_up_test_files();
//    }
//
//    virtual void TearDown() {
//        delete handler;
//    }
//
//    /**
//    * If the folder doesn't exist then make it and save a pointcloud to it.
//    * @param folder_path path to the testing folder setup.
//    */
//    void set_up_test_files() {
//        if (boost::filesystem::is_empty(folder_path)) {
//            std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> calibration = set_up_point_cloud();
//            pcl::io::savePCDFileASCII(folder_path + "/" + "test_cloud.pcd", *calibration);
//        }
//    }
//
//    static std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> set_up_point_cloud() {
//        // set up point calibration
//        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> calibration(new pcl::PointCloud<pcl::PointXYZ>);
//        calibration->height = 10;
//        calibration->width = 10;
//        calibration->is_dense = true;
//        calibration->points.resize(10 * 10);
//
//        for (int y = 0; y < 10; y++) {
//            for (int x = 0; x < 10; x++) {
//                calibration->points[y * 10 + x] = pcl::PointXYZ(x, y, 1);
//            }
//        }
//        return calibration;
//    }
//};
//
//
//TEST_F(ScanFileHandlerPhysicalFixture, TestLoadCloud) {
//    // make an empty calibration to calibration the file into.
//    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> calibration(new pcl::PointCloud<pcl::PointXYZ>);
//    // load the file
//    pcl::io::loadPCDFile<pcl::PointXYZ>(folder_path + "/" + "test_cloud.pcd", *calibration);
//    ASSERT_EQ(calibration->height, 10);
//    ASSERT_EQ(calibration->width, 10);
//    ASSERT_EQ(calibration->size(), 100);
//}
//
///**
// * Load clouds from a folder
// */
//TEST_F(ScanFileHandlerPhysicalFixture, TestLoadClouds) {
//    // make an empty calibration to calibration the file into.
//    std::vector<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>,
//            Eigen::aligned_allocator<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> > data;
//    handler->load_clouds(data, CloudType::Type::RAW);
//    ASSERT_EQ(data.size(), 4);
//
//
//}
