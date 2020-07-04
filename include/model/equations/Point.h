#ifndef SWAG_SCANNER_POINT_H
#define SWAG_SCANNER_POINT_H

#include <iostream>

namespace equations {

    /**
     * Simple object representing a point in 3d space.
     */
    class Point {
    public:
        double x,y,z;

        Point(double x, double y, double z);

        Point(std::vector<double> in);
    };
}

#endif //SWAG_SCANNER_POINT_H
