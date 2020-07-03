#ifndef SWAG_SCANNER_NORMAL_H
#define SWAG_SCANNER_NORMAL_H

#include <iostream>
#include <pcl/ModelCoefficients.h>


namespace equations {

    /**
     * Class represents the normal equation of a plane.
     */
    class Normal {
    public:
        float A, B, C;

        /**
         * Create a normal object given the coefficients of a plane.
         * @param A Coefficient
         * @param B Coefficient
         * @param C Coefficient
         */
        Normal(float A, float B, float C);

        /**
         * Initialize a normal given a vector of three coefficients.
         * @param in vector of three floats.
         * @throws invalid_argument if the vector contains too few coefficients.
         */
        Normal(std::vector<float> in);


        /**
         * Initialize a normal given PCL ModelCoefficients.
         * @param in PCL coefficients of four coefficients (A,B,C,D) but will only use A,B,C
         */
        Normal(pcl::ModelCoefficients::Ptr in);

    };
}

#endif //SWAG_SCANNER_NORMAL_H
