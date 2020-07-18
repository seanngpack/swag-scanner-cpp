#include <iostream>
#include <librealsense2/h/rs_types.h>

#ifndef SWAG_SCANNER_CAMERATYPES_H
#define SWAG_SCANNER_CAMERATYPES_H

namespace camera {
    /**
     * Intrinsics for camera.
     */
    typedef struct ss_intrinsics {
        int width;            /** width of image in pixels */
        int height;           /** height of image in pixels */
        float fx;               /** focal length of image, as a multiple of pixel width & height */
        float fy;               /** focal length of image, as a multiple of pixel width & height */
        float ppx;              /** pixel coordinates of the principal point (center of projection)  */
        float ppy;              /** pixel coordinates of the principal point (center of projection)  */
        rs2_distortion model;            /** model used to calibrate the image */
        float *coeffs;            /** coefficients describing the distortion model */
        float depth_scale;      /** multiply by camera value to get depth in meters */

        ss_intrinsics() = default;

        ss_intrinsics(rs2_intrinsics intrin, float depth_scale);

        ss_intrinsics(int width,
                      int height,
                      float fx,
                      float fy,
                      float ppx,
                      float ppy,
                      rs2_distortion model,
                      float coeffs[5],
                      float depth_scale);

        std::string to_string();

        ~ss_intrinsics();


    } ss_intrinsics;
}

#endif //SWAG_SCANNER_CAMERATYPES_H
