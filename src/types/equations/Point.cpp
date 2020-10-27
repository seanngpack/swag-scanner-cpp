#include <vector>
#include "Point.h"

equations::Point::Point(double x, double y, double z) : x(x), y(y), z(z) {}

equations::Point::Point(const std::vector<double> &in) : x(in[0]), y(in[1]), z(in[2]) {
    if (in.size() < 3) {
        throw std::invalid_argument("Cannot construct a point with fewer than three values");
    }
}
