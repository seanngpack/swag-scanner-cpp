#include "gtest/gtest.h"
#include <pcl/point_types.h>
#include "Algorithms.h"

class AlgosFixture : public ::testing::Test {

protected:
    camera::ss_intrinsics *intrinsics;
    std::vector<uint16_t> frame;

    virtual void SetUp() {
        intrinsics = new camera::ss_intrinsics(50, 50,
                                               50, 50,
                                               50, 50,
                                               "brown", 10, .01);


        for (int i = 0; i < intrinsics->width * intrinsics->height; i++) {
            frame.push_back(i);
        }
    }

    virtual void TearDown() {
        delete intrinsics;
    }


};

/**
 * Tests deprojection method to see if a point is being made correctly and see if the math
 * is good.
 */
TEST_F(AlgosFixture, TestDeproject) {
    pcl::PointXYZ actual = algos::deproject_pixel_to_point(10, 10, 100, intrinsics);

    pcl::PointXYZ expected;
    expected.x = 9;
    expected.y = 9;
    expected.z = 1;

    ASSERT_FLOAT_EQ(expected.x, actual.x);
    ASSERT_FLOAT_EQ(expected.y, actual.y);
    ASSERT_FLOAT_EQ(expected.z, actual.z);
}

/**
 * Test creating a pointcloud and see if its size, height, and width are good.
 */
TEST_F(AlgosFixture, TestCreatePC) {
    uint16_t *swag = frame.data();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = algos::create_point_cloud(swag, intrinsics);

    ASSERT_EQ(cloud->width, 50);
    ASSERT_EQ(cloud->height, 50);
    ASSERT_EQ(cloud->size(), 2500);
    ASSERT_TRUE(cloud->is_dense);
}
