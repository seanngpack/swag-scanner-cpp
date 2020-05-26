#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "Model.h"

class ModelFixture : public ::testing::Test {

protected:
    std::vector<uint16_t> depth_frame;
    const uint16_t *depth_frame_ptr;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr;
    model::Model *mod;


    virtual void SetUp() {

        mod = new model::Model;

        // Set up depth frame
        for (uint16_t i = 0; i < 100; i++) {
            depth_frame.push_back(i);
        }
        depth_frame_ptr = depth_frame.data();

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
        cloud_ptr = cloud;
    }

    virtual void TearDown() {
        delete mod;
    }
};

/**
 * Test the depth frame getter and setter.
 */
TEST_F(ModelFixture, TestDepthFrame) {
    mod->set_depth_frame(depth_frame_ptr);
    ASSERT_EQ(0, depth_frame_ptr[0]);
}

/**
 * Test throw error when getting depth frame without setting it first.
 */
TEST_F(ModelFixture, TestDepthFrameError) {
    EXPECT_THROW(mod->get_depth_frame(),
                 std::runtime_error);
}

/**
 * Test setting and getting pointcloud.
 */
TEST_F(ModelFixture, TestGetterPointCloud) {

    mod->set_point_cloud(cloud_ptr);
    EXPECT_TRUE(mod->get_point_cloud()->is_dense);
    EXPECT_EQ(mod->get_point_cloud()->width, 10);
    EXPECT_EQ(mod->get_point_cloud()->height, 10);
}

/**
 * Test getting pointcloud without having it set.
 */
TEST_F(ModelFixture, TestPointGetterCloudError) {
    EXPECT_THROW(mod->get_point_cloud(),
            std::runtime_error);
}

/**
 * Test creating a point cloud.
 */
TEST_F(ModelFixture, TestCreatePointCloud) {
    float distortion[5] = {.139, .124, .0043, .00067, -.034};
    camera::ss_intrinsics *intrinsics_distoration = new camera::ss_intrinsics(10, 10,
                                                       475.07, 475.07,
                                                       309.931, 245.011,
                                                       RS2_DISTORTION_INVERSE_BROWN_CONRADY, distortion,
                                                       0.0001);
    mod->set_depth_frame(depth_frame_ptr);
    mod->set_intrinsics(intrinsics_distoration);
    mod->create_point_cloud();
    EXPECT_EQ(mod->get_point_cloud()->width, 10);
    EXPECT_EQ(mod->get_point_cloud()->height, 10);
}

/**
 * Test creating a point cloud expecting error because the depth frame or
 * intrinsics are not set yet.
 */
TEST_F(ModelFixture, TestCreatePointCloudError) {
 EXPECT_THROW(mod->create_point_cloud(),
         std::runtime_error);
}

/**
 * Test the intrinsics getters and setters.
 */
TEST_F(ModelFixture, TestGetterIntrinsics) {
    float distortion[5] = {.139, .124, .0043, .00067, -.034};
    camera::ss_intrinsics *intrinsics_distoration = new camera::ss_intrinsics(10, 10,
                                                                              475.07, 475.07,
                                                                              309.931, 245.011,
                                                                              RS2_DISTORTION_INVERSE_BROWN_CONRADY, distortion,
                                                                              0.0001);
    mod->set_intrinsics(intrinsics_distoration);
    ASSERT_EQ(mod->get_intrinsics()->width, 10);
    ASSERT_FLOAT_EQ(mod->get_intrinsics()->ppx, 309.931);
}

/**
 * Test getting the intrinsics without setting them first, resulting in error.
 */
TEST_F(ModelFixture, TestIntrinsicsError) {
    ASSERT_THROW(mod->get_intrinsics(),
            std::runtime_error);
}

/**
 * Test creating the normals cloud.
 */
TEST_F(ModelFixture, TestEstimateNormals) {
    mod->set_point_cloud(cloud_ptr);
    mod->estimate_normal_cloud();

    ASSERT_EQ(mod->get_normal_cloud()->width, 10);
    ASSERT_EQ(mod->get_normal_cloud()->height, 10);
}
