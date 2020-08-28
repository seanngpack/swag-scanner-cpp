#ifndef SWAG_SCANNER_NORMAL_H
#define SWAG_SCANNER_NORMAL_H

#include <pcl/ModelCoefficients.h>


namespace equations {
    /**
     * Class represents the normal equation of a plane.
     */
    typedef struct Normal {
        double A = 0.0;
        double B = 0.0;
        double C = 0.0;

        /**
         * Default constructor. Don't forget to set values for A,B,C.
         */
        Normal() = default;

        /**
         * Create a normal object given the coefficients of a plane.
         * @param A Coefficient
         * @param B Coefficient
         * @param C Coefficient
         */
        Normal(double A, double B, double C);

        /**
         * Initialize a normal given a vector of three coefficients.
         * @param in vector of three floats.
         * @throws invalid_argument if the vector contains too few coefficients.
         */
        Normal(std::vector<double> in);


        /**
         * Initialize a normal given PCL ModelCoefficients.
         * @param in PCL coefficients of four coefficients (A,B,C,D) but will only use A,B,C
         */
        Normal(pcl::ModelCoefficients::Ptr in);

        /**
         * Overloaded addition + operator.
         * @param n2 other normal to add.
         * @return new normal, addition of this and other.
         */
        Normal operator+(const Normal &n2) const;

        ~Normal() = default;
    } Normal;
}

#endif //SWAG_SCANNER_NORMAL_H
