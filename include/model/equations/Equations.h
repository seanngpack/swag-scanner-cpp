#ifndef SWAG_SCANNER_EQUATIONS_H
#define SWAG_SCANNER_EQUATIONS_H

#include "Normal.h"

namespace equations {

    /**
     * Cross product of two 1x3 Normals.
     * @param n1 first normal.
     * @param n2 second normal.
     * @return cross product result.
     */
    inline Normal cross(Normal n1, Normal n2) {
        return Normal(-n2.B * n1.C + n1.B * n2.C, n2.A * n1.C - n1.A * n2.C, -n2.A * n1.B + n1.A * n2.B);
    }

    /**
     * Take the norm of given normal.
     * @param n the normal.
     * @return scalar result of norm.
     */
    inline double norm(Normal n) {
        return sqrt(pow(n.A, 2) + pow(n.B, 2) + pow(n.C, 2));
    }

    /**
     * Calculate norm(n1)/norm(cross(n1,n2))
     * @param n1 normal.
     * @param n2 normal.
     * @return norm(n1)/norm(cross(n1,n2)).
     */
    inline double coeff(Normal n1, Normal n2) {
        return norm(n1) / norm(cross(n1, n2));
    }
}

#endif //SWAG_SCANNER_EQUATIONS_H