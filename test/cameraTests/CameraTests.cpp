#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "SR305.h"
#include "CameraTypes.h"
#include <memory>

/**
 * TODO: implement these tests for the camera, making sure to test pointcloud creation
 */

//class CameraFixture : public ::testing::Test {
//
//protected:
//    std::vector<uint16_t> depth_frame;
//    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_ptr;
//
//
//    virtual void SetUp() {
//
//        mod = new model::Model;
//
//        // Set up depth frame
//        for (uint16_t i = 0; i < 100; i++) {
//            depth_frame.push_back(i);
//        }
//
//        // set up point calibration
//        auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
//        cloud->height = 10;
//        cloud->width = 10;
//        cloud->is_dense = true;
//        cloud->points.resize(10 * 10);
//
//        for (int y = 0; y < 10; y++) {
//            for (int x = 0; x < 10; x++) {
//                cloud->points[y * 10 + x] = pcl::PointXYZ(x, y, 1);
//            }
//        }
//        cloud_ptr = cloud;
//    }
//
//    float distortion[5] = {.139, .124, .0043, .00067, -.034};
//    camera::intrinsics *intrinsics_distortion = new camera::intrinsics(10, 10,
//                                                                       475.07, 475.07,
//                                                                       309.931, 245.011,
//                                                                       RS2_DISTORTION_INVERSE_BROWN_CONRADY,
//                                                                       distortion,
//                                                                       0.0001);
//
//    virtual void TearDown() {
//        delete mod;
//    }
//};
//
//
///**
// * Test creating a point calibration.
// */
//TEST_F(CameraFixture, TestCreatePointCloud) {
//
//
//    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> test = mod->create_point_cloud(depth_frame, *intrinsics_distortion);
//    EXPECT_EQ(test->width, 10);
//    EXPECT_EQ(test->height, 10);
//}
//
///**
// * Test creating the normals calibration.
// */
//TEST_F(CameraFixture, TestEstimateNormals) {
//    auto test = mod->create_point_cloud(depth_frame, *intrinsics_distortion);
//    mod->estimate_normal_cloud(test);
//
//    ASSERT_EQ(test->width, 10);
//    ASSERT_EQ(test->height, 10);
//}


