/**
 * Structs for the camera.z
 */

#ifndef SWAG_SCANNER_CAMERATYPES_H
#define SWAG_SCANNER_CAMERATYPES_H

namespace camera {
    /**
     * Intrinsics for camera.
     */
    typedef enum ss_intrinsics {
        width,            /** width of image in pixels */
        height,           /** height of image in pixels */
        fx,               /** focal length of image, as a multiple of pixel width & height */
        fy,               /** focal length of image, as a multiple of pixel width & height */
        ppx,              /** pixel coordinates of the principal point (center of projection)  */
        ppy,              /** pixel coordinates of the principal point (center of projection)  */
        model,            /** model used to calibrate the image */
        coeffs            /** coefficients describing the distortion model */
    } ss_intrinsics;
}

#endif //SWAG_SCANNER_CAMERATYPES_H
