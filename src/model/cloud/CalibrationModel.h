#ifndef SWAG_SCANNER_CALIBRATIONMODEL_H
#define SWAG_SCANNER_CALIBRATIONMODEL_H

#include "IModel.h"
#include <Eigen/Dense>

namespace file {
    class CalibrationFileHandler;
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
         * Set the calibration name.
         *
         * @param cal_name calibration name.
         */
        void set_calibration(const std::string &cal_name);

        /**
         * Load clouds from latest calibration.
         */
        void load_clouds();

        /**
         * Load clouds given calibration name.
         *
         * @param cal_name name of calibration you want to load clouds from.
         */
        void load_clouds(const std::string &cal_name);


    private:
        std::unique_ptr<file::CalibrationFileHandler> file_handler;

        /**
         * Get the upright and ground plane equations.
         * Utilizes a hardcoded axis and RANSAC normals to find the ground plane. It finds a plane that is within
         * the epsilon angle deviation to robustly find the ground plane. Then it will calculate the normals
         * and find the perpendicular plane which should be the upright pane.
         *
         * @param cloud calibration cloud.
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

        /**
         * Calculate the center point of the turntable.
         * Solve for x using in Ax = b using SVD.
         *
         * @param axis_dir axis of rotation direction.
         * @param upright_planes use the upright planes in calculation.
         * @return center point of turntable from the camera origin in meters.
         */
        pcl::PointXYZ calculate_center_pt(const equations::Normal &axis_dir,
                                          const std::vector<equations::Plane> &upright_planes);


        /**
         * Project center point to ground plane.
         *
         * @param cloud the cloud that the plane you are projecting to belongs to.
         * @param pt point you want to project.
         * @param plane the plane you want to project onto.
         * @param delta the threshold to search for point on plane.
         * @return projected point on the plane.
         */
        pcl::PointXYZ refine_center_pt(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud,
                                       const pcl::PointXYZ &pt,
                                       const equations::Plane &plane,
                                       double delta = .00001
        );

    };


}

#endif //SWAG_SCANNER_CALIBRATIONMODEL_H
