#ifndef SWAG_SCANNER_CALIBRATION_H
#define SWAG_SCANNER_CALIBRATION_H

#include "Normal.h"
#include "Plane.h"
#include "Equations.h"
#include "Point.h"

namespace calibration {

    /**
     * Solve for x using in Ax = b using SVD.
     * @param A lhs matrix.
     * @param b rhs matrix.
     * @return origin point of the turntable in m.
     */
    inline pcl::PointXYZ calculate_center_pt(const Eigen::MatrixXd &A, const Eigen::MatrixXd &b) {
        Eigen::MatrixXd sol_mat = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
        std::vector<double> sol_vec(sol_mat.data(), sol_mat.data() + sol_mat.rows() * sol_mat.cols());
        return pcl::PointXYZ(sol_vec[0], sol_vec[1], sol_vec[2]);
    }

    /**
      * Calculate the A matrix in Ax = b
      * @param g_n normal vector to the ground plane.
      * @param upright_planes vector of plane equations for the upright planes.
      * @return the A matrix (N-1, 3)
      */
    inline Eigen::MatrixXd build_A_matrix(const equations::Normal &g_n,
                                          const std::vector<equations::Plane> &upright_planes) {
        int rows = upright_planes.size() - 1;
        Eigen::MatrixXd A(rows, 3);
        for (int i = 0; i < rows; i++) {
            A(i, 0) = equations::coeff(g_n, upright_planes[i].get_normal()) * upright_planes[i].A -
                      equations::coeff(g_n, upright_planes[i + 1].get_normal()) * upright_planes[i + 1].A;
            A(i, 1) = equations::coeff(g_n, upright_planes[i].get_normal()) * upright_planes[i].B -
                      equations::coeff(g_n, upright_planes[i + 1].get_normal()) * upright_planes[i + 1].B;
            A(i, 2) = equations::coeff(g_n, upright_planes[i].get_normal()) * upright_planes[i].C -
                      equations::coeff(g_n, upright_planes[i + 1].get_normal()) * upright_planes[i + 1].C;
        }

        return A;
    }

    /**
      * Calculate the b matrix in Ax = b
      * @param g_n normal vector to the ground plane.
      * @param upright_planes vector of plane equations for the upright planes.
      * @return the b matrix (1, N-1)
      */
    inline Eigen::MatrixXd build_b_matrix(const equations::Normal &g_n,
                                          const std::vector<equations::Plane> &upright_planes) {
        int rows = upright_planes.size() - 1;
        Eigen::MatrixXd b(rows, 1);
        for (int i = 0; i < rows; i++) {
            b(i, 0) = equations::coeff(g_n, upright_planes[i + 1].get_normal()) * upright_planes[i + 1].D -
                      equations::coeff(g_n, upright_planes[i].get_normal()) * upright_planes[i].D;
        }
        return b;
    }


}

#endif //SWAG_SCANNER_CALIBRATION_H
