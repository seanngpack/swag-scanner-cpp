#ifndef SWAG_SCANNER_CALIBRATION_H
#define SWAG_SCANNER_CALIBRATION_H

#include "Normal.h"
#include "Plane.h"
#include "Equations.h"

namespace calibration {

/**
  * Calculate the A matrix in Ax = b
  * @param g_n normal vector to the ground plane.
  * @param upright_planes vector of plane equations for the upright planes.
  * @return the A matrix (N-1, 3)
  */
    inline Eigen::MatrixXf build_A_matrix(equations::Normal g_n,
                                          std::vector<equations::Plane> upright_planes) {
        int rows = upright_planes.size() - 1;
        int cols = 3;
        Eigen::MatrixXd A(rows, cols);
        for (int i = 0; i < rows - 1; i++) {
            A(i, 0) = equations::coeff(g_n, upright_planes[i].get_normal()) * upright_planes[i].A -
                      equations::coeff(g_n, upright_planes[i + 1].get_normal()) * upright_planes[i + 1].A;
            A(i, 1) = equations::coeff(g_n, upright_planes[i].get_normal()) * upright_planes[i].B -
                      equations::coeff(g_n, upright_planes[i + 1].get_normal()) * upright_planes[i + 1].B;
            A(i, 2) = equations::coeff(g_n, upright_planes[i].get_normal()) * upright_planes[i].C -
                      equations::coeff(g_n, upright_planes[i + 1].get_normal()) * upright_planes[i + 1].C;
        }
    }


}

#endif //SWAG_SCANNER_CALIBRATION_H
