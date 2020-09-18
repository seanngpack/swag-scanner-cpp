#ifndef SWAG_SCANNER_POINT_H
#define SWAG_SCANNER_POINT_H

#include <iostream>

namespace equations {

    /**
     * Simple object representing a point in 3d space.
     */
    typedef struct Point {
    public:
        double x,y,z;

        Point(double x, double y, double z);

        Point(const std::vector<double> &in);

        ~Point() = default;
    } Point;
}

#endif //SWAG_SCANNER_POINT_H
