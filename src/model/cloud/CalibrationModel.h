#ifndef SWAG_SCANNER_CALIBRATIONMODEL_H
#define SWAG_SCANNER_CALIBRATIONMODEL_H

#include "IModel.h"
#include "Normal.h"
#include "Plane.h"
#include "Equations.h"

namespace file {
    class CalibrationFileHandler;
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
         * Solve for x using in Ax = b using SVD.
         * @param A lhs matrix.
         * @param b rhs matrix.
         * @return origin point of the turntable in m.
         */
        pcl::PointXYZ calculate_center_pt(const Eigen::MatrixXd &A, const Eigen::MatrixXd &b);

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


    };
}

#endif //SWAG_SCANNER_CALIBRATIONMODEL_H
