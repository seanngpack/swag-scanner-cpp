#include "../../../include/model/Equations/Plane.h"


equations::Plane::Plane(float A, float B, float C, float D) : A(A), B(B), C(C), D(D) {}

equations::Plane::Plane(std::vector<float> in) : A(in[0]), B(in[1]), C(in[2]), D(in[3]) {}

equations::Plane::Plane(pcl::ModelCoefficients::Ptr in) : A(in->values[0]), B(in->values[1]),
                                                          C(in->values[2]), D(in->values[3]) {}

equations::Normal equations::Plane::get_normal() {
    return equations::Normal(this->A, this->B, this->C);
}


