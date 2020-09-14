#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <filesystem>
#include <iostream>
#include <pcl/registration/icp.h>
#include <pcl/registration/ia_ransac.h>
#include <vector>
#include <memory>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc
#include "Visualizer.h"
#include "Model.h"
#include "Normal.h"
#include "Plane.h"
#include "Point.h"
#include "ScanFileHandler.h"
#include "Algorithms.h"
#include "Constants.h"
#include "CalibrationFileHandler.h"

namespace fs = std::filesystem;

class RegistrationFixture : public ::testing::Test {

protected:
    model::Model *mod;
    visual::Visualizer *viewer;

    virtual void SetUp() {
        mod = new model::Model();
        viewer = new visual::Visualizer();
    }

    virtual void TearDown() {
        delete mod;
        delete viewer;
    }
};

/**
 * This method is to test processing and registration algorithms.
 * Will remove processing once i figure out a good processing scheme.
 * Config settings:
 * "decimation_magnitude": 2,
    "spatial_filter_magnitude": 5,
    "spatial_smooth_alpha": 0.6,
    "spatial_smooth_delta": 5
 */

TEST_F(RegistrationFixture, TestRegistration) {
    using namespace constants;

    auto fixture_raw = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::io::loadPCDFile<pcl::PointXYZ>(fs::current_path().string() + "/research/registration/data/0.pcd",
                                        *fixture_raw);
    auto *file_handler = new file::ScanFileHandler();
    equations::Normal rot_axis(0.002451460662859972, -0.8828002989292145, -0.4696775645017624);
    pcl::PointXYZ center_pt(-0.005918797571212053, 0.06167422607541084, 0.42062291502952576);
    std::vector<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> cropped_clouds;
    std::vector<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> world_clouds;
    auto clouds = file_handler->load_clouds(CloudType::Type::FILTERED);
    for (const auto &c :clouds) {
        cropped_clouds.push_back(mod->crop_cloud(c, cal_min_x, cal_max_x, cal_min_y, cal_max_y, cal_min_z, cal_max_z));
    }

    for (const auto &c : cropped_clouds) {
        auto transformed = mod->transform_cloud_to_world(c, center_pt, rot_axis);
        auto transformed_cropped = mod->crop_cloud(transformed, scan_min_x, scan_max_x, scan_min_y, scan_max_y,
                                                   scan_min_z, scan_max_z);
        world_clouds.push_back(transformed_cropped);
//        viewer->simpleVis(transformed_cropped);
    }

//    file_handler->save_cloud(world_clouds[0], "0.pcd", CloudType::Type::PROCESSED);
//    file_handler->save_cloud(world_clouds[1], "20.pcd", CloudType::Type::PROCESSED);


    auto global_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    *global_cloud = *world_clouds[0];
    pcl::PointCloud<pcl::PointXYZ> rotated;
    for (int i = 1; i < world_clouds.size(); i++) {
        rotated = mod->rotate_cloud_about_z_axis(world_clouds[i], 20 * i);
        *global_cloud += rotated;
        rotated.clear();
    }

    viewer->simpleVis(global_cloud);

}

bool next_iteration = false;
void
print4x4Matrix (const Eigen::Matrix4d & matrix)
{
    printf ("Rotation matrix :\n");
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
    printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
    printf ("Translation vector :\n");
    printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

void
keyboardEventOccurred (const pcl::visualization::KeyboardEvent& event,
                       void* nothing)
{
    if (event.getKeySym () == "space" && event.keyDown ())
        next_iteration = true;
}

/**
 * Test ICP step by step
 */
TEST_F(RegistrationFixture, TestICPStepByStep) {
    typedef pcl::PointXYZ PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;

    auto cloud_in = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto cloud_tr = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto cloud_icp = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::io::loadPCDFile<pcl::PointXYZ>(fs::current_path().string() + "/research/calibration/data/0.pcd",
                                        *cloud_in);
    pcl::io::loadPCDFile<pcl::PointXYZ>(fs::current_path().string() + "/research/calibration/data/20.pcd",
                                        *cloud_tr);
    *cloud_tr = mod->rotate_cloud_about_z_axis(cloud_tr, 20);



    int iterations = 100;  // Default number of ICP iterations

    pcl::console::TicToc time;
    time.tic ();

    // Defining a rotation matrix and translation vector
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();

    // The Iterative Closest Point algorithm
    time.tic ();
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource (cloud_icp);
    icp.setInputTarget (cloud_in);
    icp.setMaximumIterations (iterations);
    icp.setTransformationEpsilon (1e-8);
    icp.setMaxCorrespondenceDistance (0.5);
    icp.setEuclideanFitnessEpsilon (1);
    icp.align (*cloud_icp);

    std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc () << " ms" << std::endl;

    if (icp.hasConverged ())
    {
        std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
        std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
        transformation_matrix = icp.getFinalTransformation ().cast<double>();
        print4x4Matrix (transformation_matrix);
    }
    else
    {"/"
     ""
     "   "
        PCL_ERROR ("\nICP has not converged.\n");
    }

    // Visualization
    pcl::visualization::PCLVisualizer viewer ("ICP demo");
    // Create two vertically separated viewports
    int v1 (0);
    int v2 (1);
    viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);

    // The color we will be using
    float bckgr_gray_level = 0.0;  // Black
    float txt_gray_lvl = 1.0 - bckgr_gray_level;

    // Original point cloud is white
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h (cloud_in, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl,
                                                                               (int) 255 * txt_gray_lvl);
    viewer.addPointCloud (cloud_in, cloud_in_color_h, "cloud_in_v1", v1);
    viewer.addPointCloud (cloud_in, cloud_in_color_h, "cloud_in_v2", v2);

    // Transformed point cloud is green
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h (cloud_tr, 20, 180, 20);
    viewer.addPointCloud (cloud_tr, cloud_tr_color_h, "cloud_tr_v1", v1);

    // ICP aligned point cloud is red
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h (cloud_icp, 180, 20, 20);
    viewer.addPointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2", v2);

    // Adding text descriptions in each viewport
    viewer.addText ("White: Original point cloud\nGreen: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
    viewer.addText ("White: Original point cloud\nRed: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

    std::stringstream ss;
    ss << iterations;
    std::string iterations_cnt = "ICP iterations = " + ss.str ();
    viewer.addText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);

    // Set background color
    viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
    viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

    // Set camera position and orientation
    viewer.setCameraPosition (-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
    viewer.setSize (1280, 1024);  // Visualiser window size

    // Register keyboard callback :
    viewer.registerKeyboardCallback (&keyboardEventOccurred, (void*) NULL);

    // Display the visualiser
    while (!viewer.wasStopped ())
    {
        viewer.spinOnce ();

        // The user pressed "space" :
        if (next_iteration)
        {
            // The Iterative Closest Point algorithm
            time.tic ();
            icp.align (*cloud_icp);
            std::cout << "Applied 1 ICP iteration in " << time.toc () << " ms" << std::endl;

            if (icp.hasConverged ())
            {
                printf ("\033[11A");  // Go up 11 lines in terminal output.
                printf ("\nICP has converged, score is %+.0e\n", icp.getFitnessScore ());
                std::cout << "\nICP transformation " << ++iterations << " : cloud_icp -> cloud_in" << std::endl;
                transformation_matrix *= icp.getFinalTransformation ().cast<double>();  // WARNING /!\ This is not accurate! For "educational" purpose only!
                print4x4Matrix (transformation_matrix);  // Print the transformation between original pose and current pose

                ss.str ("");
                ss << iterations;
                std::string iterations_cnt = "ICP iterations = " + ss.str ();
                viewer.updateText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
                viewer.updatePointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2");
            }
            else
            {
                PCL_ERROR ("\nICP has not converged.\n");
            }
        }
        next_iteration = false;
    }


}