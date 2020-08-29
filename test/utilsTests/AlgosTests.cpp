#include "gtest/gtest.h"
#include "Algorithms.h"
#include "Visualizer.h"
#include "Model.h"
#include "CameraTypes.h"
#include <pcl/point_types.h>
#include <librealsense2/h/rs_types.h>
#include <pcl/common/impl/transforms.hpp>
#include <pcl/io/pcd_io.h>



class AlgosFixture : public ::testing::Test {

protected:
    camera::ss_intrinsics *intrinsics_no_distoration;
    camera::ss_intrinsics *intrinsics_distoration;
    std::vector<uint16_t> frame;

    virtual void SetUp() {
        float no_distortion[5] = {0, 0, 0, 0, 0};
        float distortion[5] = {.139, .124, .0043, .00067, -.034};
        intrinsics_no_distoration = new camera::ss_intrinsics(640, 480,
                                                              475.07, 475.07,
                                                              309.931, 245.011,
                                                              RS2_DISTORTION_INVERSE_BROWN_CONRADY,
                                                              no_distortion,
                                                              0.0001);
        intrinsics_distoration = new camera::ss_intrinsics(640, 480,
                                                           475.07, 475.07,
                                                           309.931, 245.011,
                                                           RS2_DISTORTION_INVERSE_BROWN_CONRADY, distortion,
                                                           0.0001);


        for (float i = 0; i < intrinsics_no_distoration->width * intrinsics_no_distoration->height; i++) {
            frame.push_back(1);
        }
    }

    virtual void TearDown() {
        delete intrinsics_no_distoration;
        delete intrinsics_distoration;
    }


};

/**
 * Tests deprojection method to see if a point is being made correctly and see if the math
 * is good.
 */
TEST_F(AlgosFixture, TestDeprojectNoDistortion) {
    pcl::PointXYZ actual = algos::deproject_pixel_to_point(10, 10, 100, intrinsics_no_distoration);

    pcl::PointXYZ expected;
    expected.x = -.0063134059;
    expected.y = -.0049468707;
    expected.z = .01;

    ASSERT_FLOAT_EQ(expected.x, actual.x);
    ASSERT_FLOAT_EQ(expected.y, actual.y);
    ASSERT_FLOAT_EQ(expected.z, actual.z);
}

/**
 * Tests deprojection method with distortion coefficients.
 */
TEST_F(AlgosFixture, TestDeprojectDistortion) {
    pcl::PointXYZ actual = algos::deproject_pixel_to_point(10, 10, 100, intrinsics_distoration);

    pcl::PointXYZ expected;
    expected.x = -.0063134059;
    expected.y = -.0049468707;
    expected.z = .01;

    ASSERT_FLOAT_EQ(expected.x, actual.x);
    ASSERT_FLOAT_EQ(expected.y, actual.y);
    ASSERT_FLOAT_EQ(expected.z, actual.z);
}

/**
 * Given center of bed point, axis of rotation, transform the cloud into the world
 * coordinate frame!!!
 * Then visualize it.
 */
TEST_F(AlgosFixture, TestTransformCoordinate) {

    std::string folder_path = "/Users/seanngpack/Library/Application Support/SwagScanner/calibration/test5/12.pcd";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr result_cropped(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ pt(-0.006283042311759926,
                     0.014217784268003741,
                     0.4304016110847342);

    pcl::io::loadPCDFile<pcl::PointXYZ>(folder_path, *cloudIn);

    //point -> origin so I flipped the signs
    // NOTE I FLIPPED THE SIGNS OF ORIGINAL
    Eigen::Vector3f trans_vec(0.006283042311759926,
                              -0.014217784268003741,
                              -0.4304016110847342);
    Eigen::Translation<float, 3> translation(trans_vec);

    float a_dot_b = Eigen::Vector3f(-0.0020733693898364438,
                                    -0.8143288642168045,
                                    -0.5803786158561707).dot(
            Eigen::Vector3f(0, 0, 1));
    float angle = acos(a_dot_b);
    std::cout << (180 / 3.1415) * angle << std::endl;
    // rotate about the x axis to align the z axis together
    Eigen::AngleAxis<float> rot(-angle, Eigen::Vector3f(1,
                                                        0,
                                                        0));

    // apply A onto B
    Eigen::Transform<float, 3, Eigen::Affine> combined =
            rot * translation;
    std::cout << combined.matrix() << std::endl;
    pcl::transformPointCloud(*cloudIn, *result, combined.matrix());

    model::Model model;
    result_cropped = model.crop_cloud(result, -.089, .089,
                                      -.089, .089,
                                      -.03, .05);

    visual::Visualizer visualizer;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> clouds{cloudIn, result};
//    std::vector<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> clouds{result, result_cropped};
    visualizer.simpleVisColor(clouds);
//    visualizer.ptVis(cloudIn, pt);
}


