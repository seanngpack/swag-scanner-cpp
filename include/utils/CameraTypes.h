/**
 * Structs for the camera.z
 */

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

        ss_intrinsics(int width,
                      int height,
                      float fx,
                      float fy,
                      float ppx,
                      float ppy,
                      rs2_distortion model,
                      float coeffs[5],
                      float depth_scale) : width(width), height(height), fx(fx),
                                           fy(fy), ppx(ppx), ppy(ppy), model(model),
                                           coeffs(coeffs), depth_scale(depth_scale) {

        };

        ~ss_intrinsics() {
            std::cout << "destroying camera intrinsics...\n";
        }

        std::string toString() {
            return "width: " + std::to_string(width) + "\n" +
                   "height: " + std::to_string(height) + "\n" +
                   "fx: " + std::to_string(fx) + "\n" +
                   "fy: " + std::to_string(fy) + "\n" +
                   "ppx: " + std::to_string(ppx) + "\n" +
                   "ppy: " + std::to_string(ppy) + "\n" +
                    "depth scale: " + std::to_string(depth_scale);
        }
    } ss_intrinsics;
}

#endif //SWAG_SCANNER_CAMERATYPES_H
