#include "Plane.h"

equations::Plane::Plane(double A, double B, double C, double D) : A(A), B(B), C(C), D(D) {}

equations::Plane::Plane(std::vector<double> in) : A(in[0]), B(in[1]), C(in[2]), D(in[3]) {}

equations::Plane::Plane(pcl::ModelCoefficients::Ptr in) : A(in->values[0]), B(in->values[1]),
                                                          C(in->values[2]), D(in->values[3]) {}

equations::Normal equations::Plane::get_normal() {
    return equations::Normal(this->A, this->B, this->C);
}

equations::Plane equations::Plane::operator+(const equations::Plane &p2) const {
    return equations::Plane(A + p2.A, B + p2.B, C + p2.C, D + p2.D);
}


