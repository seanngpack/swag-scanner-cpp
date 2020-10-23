#ifndef SWAG_SCANNER_CALIBRATIONMODEL_H
#define SWAG_SCANNER_CALIBRATIONMODEL_H

#include "IModel.h"
#include "CalibrationFileHandler.h"
#include "Normal.h"
#include <pcl/point_types.h>
#include <Eigen/Dense>

namespace file {
    class CalibrationFileHandler;
}

namespace spdlog {
    class logger;
}

namespace equations {
    class Normal;

    class Point;

    class Plane;
}

namespace model {
    /**
     * Represents a calibration model to do calibration processing on the clouds.
     */
    class CalibrationModel : public IModel {

    public:

        /**
         * Initialize file handler in here.
         */
        CalibrationModel();

        /**
         * Set the calibration name. This triggers the filehandler to set the current working directory
         * to the given input. This will also clear any existing clouds in the model.
         *
         * @param cal_name calibration name.
         */
        void set_calibration(const std::string &cal_name);

        /**
         * Save calibration to the current calibration folder.
         */
        void save_cloud(const std::string &cloud_name);


        /**
         * Calculate the center point of the turntable.
         * Using vector of clouds, find the ground and upright planes, get the axis of rotation,
         * build A and b matrices, and finally solve for the center point x using in Ax = b using SVD.
         *
         * @param axis_dir axis of rotation direction.
         * @param upright_planes use the upright planes in calculation.
         * @return center point of turntable from the camera origin in meters.
         */
        pcl::PointXYZ calculate_center_point();

        /**
         * Overloaded method also accepts axis direction planes instead of using model's.
         *
         * @param axis_dir direction of rotation axis.
         * @param upright_planes vector of upright plane equations.
         * @return
         */
        pcl::PointXYZ calculate_center_point(const equations::Normal &axis_dir,
                                             const std::vector<equations::Plane> &upright_planes);


        /**
         * Project center point to ground plane.
         *
         * @param cloud the calibration that the plane you are projecting to belongs to.
         * @param pt point you want to project.
         * @param delta the threshold to search for point on plane.
         * @return projected point on the plane.
         * @throws if the center point has not been calculated yet.
         */
        pcl::PointXYZ refine_center_point(double delta = .00001);

        /**
         * Update calibration json with axis of rotation and center point info.
         */
        void update_calibration_json();


    private:
        file::CalibrationFileHandler file_handler;
        std::vector<equations::Plane> ground_planes;
        std::vector<equations::Plane> upright_planes;
        pcl::PointXYZ center_point;
        equations::Normal axis_of_rotation;


        /**
         * Get the upright and ground plane equations.
         * Utilizes a hardcoded axis and RANSAC normals to find the ground plane. It finds a plane that is within
         * the epsilon angle deviation to robustly find the ground plane. Then it will calculate the normals
         * and find the perpendicular plane which should be the upright pane.
         *
         * @param cloud calibration calibration.
         * @param visual_flag flag whether to visualize segmentation or not.
         * @return vector of upright and ground plane equations.
         */
        std::vector<equations::Plane>
        get_calibration_planes_coefs(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud,
                                     bool visual_flag = false);

        /**
          * Calculate the A matrix in Ax = b
          * @param g_n normal vector to the ground plane.
          * @param upright_planes vector of plane equations for the upright planes.
          * @return the A matrix (N-1, 3)
          */
        Eigen::MatrixXd build_A_matrix(const equations::Normal &g_n,
                                       const std::vector<equations::Plane> &upright_planes);

        /**
         * Calculate the b matrix in Ax = b
         * @param g_n normal vector to the ground plane.
         * @param upright_planes vector of plane equations for the upright planes.
         * @return the b matrix (1, N-1)
         */
        Eigen::MatrixXd build_b_matrix(const equations::Normal &g_n,
                                       const std::vector<equations::Plane> &upright_planes);

        /**
         * Given a vector of ground planes, get the average of their normals to get the rotation axis direction.
         *
         * @param ground_planes planes to calculate wiht.
         * @return rotation axis direction.
         */
        equations::Normal calculate_axis_dir(const std::vector<equations::Plane> &ground_planes);


    };


}

#endif //SWAG_SCANNER_CALIBRATIONMODEL_H
