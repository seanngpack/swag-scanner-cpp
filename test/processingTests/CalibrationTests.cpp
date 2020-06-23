#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "Model.h"
#include "Visualizer.h"
#include "Segmentation.h"


class CalibrationPhysicalFixture : public ::testing::Test {

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


TEST_F(CalibrationPhysicalFixture, TestPlaneCalibration) {

    std::string folder_path = "/Users/seanngpack/Programming Stuff/Projects/scanner_files/18";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>(folder_path + "/filtered/" + "1.pcd", *cloudIn);
    pcl::io::loadPCDFile<pcl::PointXYZ>(folder_path + "/filtered/" + "4.pcd", *cloudOut);
//    std::vector<float> coefs = mod->get_plane_coefs(cloudIn);
//    mod->get_plane_coefs(cloudOut);

    std::vector<float> coefs = {-0.2685, -14.7235, -8.4938};

    // from 1.pcd -> 4.pcd
    float theta = 0.15708;
    std::vector<float> temp = {coefs[0], coefs[1], coefs[2]};
    Eigen::Vector3f axis(temp.data());

//    Eigen::Matrix3f c_matrix;
//    c_matrix << 0, -axis[2], axis[1],
//            axis[2], 0, -axis[0],
//            -axis[1], axis[0], 0;

//    Eigen::Matrix3f iden = Eigen::Matrix3f::Identity();
//    Eigen::Matrix3f R = iden + (1 - cos(theta)) * c_matrix * c_matrix + sin(theta) * c_matrix;
//    std::cout << R << std::endl;
//
//    Eigen::Matrix4f rot_matrix;
//    rot_matrix << R(0, 0), R(0, 1), R(0, 2), 0,
//            R(1, 0), R(1, 1), R(1, 2), 0,
//            R(2, 0), R(2, 1), R(2, 2), 0,
//            0, 0, 0, 1;
//    std::cout << rot_matrix << std::endl;
//    pcl::transformPointCloud(*cloudIn, *transformed, rot_matrix);
//
//    pcl::PointCloud<pcl::PointXYZ>::Ptr transformedICP(new pcl::PointCloud<pcl::PointXYZ>);
//    mod->register_pair_clouds(transformed, cloudOut, transformedICP);

    visual::Visualizer visualizer;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> clouds{cloudOut, transformedICP};
    visualizer.simpleVis(clouds);

}