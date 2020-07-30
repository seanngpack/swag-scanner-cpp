#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "Model.h"


class ModelFixture : public ::testing::Test {

protected:
    std::vector<uint16_t> depth_frame;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr;
    model::Model *mod;


    virtual void SetUp() {

        mod = new model::Model;

        // Set up depth frame
        for (uint16_t i = 0; i < 100; i++) {
            depth_frame.push_back(i);
        }

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
 * Test creating a point cloud.
 */
TEST_F(ModelFixture, TestCreatePointCloud) {
    float distortion[5] = {.139, .124, .0043, .00067, -.034};
    camera::ss_intrinsics *intrinsics_distortion = new camera::ss_intrinsics(10, 10,
                                                                             475.07, 475.07,
                                                                             309.931, 245.011,
                                                                             RS2_DISTORTION_INVERSE_BROWN_CONRADY,
                                                                             distortion,
                                                                             0.0001);

    pcl::PointCloud<pcl::PointXYZ>::Ptr test = mod->create_point_cloud(depth_frame, *intrinsics_distortion);
    EXPECT_EQ(test->width, 10);
    EXPECT_EQ(test->height, 10);
}

/**
 * Test creating the normals cloud.
 */
TEST_F(ModelFixture, TestEstimateNormals) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr test(new pcl::PointCloud<pcl::PointXYZ>);
    mod->estimate_normal_cloud(test);

    ASSERT_EQ(test->width, 10);
    ASSERT_EQ(test->height, 10);
}


