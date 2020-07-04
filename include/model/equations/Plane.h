#ifndef SWAG_SCANNER_PLANE_H
#define SWAG_SCANNER_PLANE_H

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include "Normal.h"

namespace equations {

    /**
     * Class represents the equation of a plane. Contains methods to convert vector or pcl::ModelCoefficients
     * to this object.
     */
    class Plane {
    public:

        double A, B, C, D;

        /**
         * Create a plane object given the coefficients of a plane.
         * @param A Coefficient
         * @param B Coefficient
         * @param C Coefficient
         * @param D Coefficient
         */
        Plane(double A, double B, double C, double D);

        /**
         * Initialize a plane given a vector.
         * @param in vector of four coefficients (A,B,C,D)
         * @throws invalid_argument if the vector is missing a coefficient.
         */
        Plane(std::vector<double> in);

        /**
         * Initialize a plane given PCL ModelCoefficients.
         * @param in PCL coefficients of four coefficients (A,B,C,D)
         */
        Plane(pcl::ModelCoefficients::Ptr in);

        /**
         * Generate normal from this plane.
         * @return the normal.
         */
        Normal get_normal();

    private:

    };
}

#endif //SWAG_SCANNER_PLANE_H
