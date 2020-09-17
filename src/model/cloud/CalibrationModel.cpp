#include "CalibrationModel.h"
#include "CalibrationFileHandler.h"

model::CalibrationModel::CalibrationModel() :
        file_handler(std::make_unique<file::CalibrationFileHandler>()) {}

void model::CalibrationModel::set_calibration(const std::string &cal_name) {
    file_handler->set_calibration(cal_name);
}

void model::CalibrationModel::load_clouds() {
//    clouds = file_handler->load_clouds();
// TODO: modify load_clouds in calibrationFileHandler to have a default laod_clouds
// that loads form latest
}

void model::CalibrationModel::load_clouds(const std::string &cal_name) {
// TODO: overload load_clouds in calibrationFileHandler to accept a file name
}






// --------------------------------------------------------------------------------
//                          PRIVATE METHODS
// --------------------------------------------------------------------------------


pcl::PointXYZ model::CalibrationModel::calculate_center_pt(const Eigen::MatrixXd &A, const Eigen::MatrixXd &b) {
    Eigen::MatrixXd sol_mat = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
    std::vector<double> sol_vec(sol_mat.data(), sol_mat.data() + sol_mat.rows() * sol_mat.cols());
    return pcl::PointXYZ(sol_vec[0], sol_vec[1], sol_vec[2]);
}

Eigen::MatrixXd model::CalibrationModel::build_A_matrix(const equations::Normal &g_n,
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

Eigen::MatrixXd model::CalibrationModel::build_b_matrix(const equations::Normal &g_n,
                               const std::vector<equations::Plane> &upright_planes) {
    int rows = upright_planes.size() - 1;
    Eigen::MatrixXd b(rows, 1);
    for (int i = 0; i < rows; i++) {
        b(i, 0) = equations::coeff(g_n, upright_planes[i + 1].get_normal()) * upright_planes[i + 1].D -
                  equations::coeff(g_n, upright_planes[i].get_normal()) * upright_planes[i].D;
    }
    return b;
}