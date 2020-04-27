#ifndef SWAG_SCANNER_CPP_ICAMERA_H
#define SWAG_SCANNER_CPP_ICAMERA_H

#include <vector>
#include <iostream>
#include <librealsense2/rs.hpp>
#include "CameraTypes.h"


namespace camera {
    /**
     * Interface for a camera. Contains methods to get depth maps and return them as 1d vectors.
     */
    class ICamera {
    public:

        /**
         * Get pointer to the camera intrinsics and store them to class fields.
         * @return a pointer to ss_camera struct of the camera intrinsics.
         */
        virtual camera::ss_intrinsics *get_instrinsics() = 0;

        /**
         * Get depth frame which is a pointer to array of uint16_t.
         * @return the depth map.
         */
        virtual const uint16_t *get_depth_frame() = 0;

        /**
         * Virtual destructor, must be defined or else it will never call the base class's destructor.
         */
        virtual ~ICamera() {
            std::cout << "calling ICamera destructor \n";
        }


    protected:
        camera::ss_intrinsics intrinsics;
    };

}


#endif //SWAG_SCANNER_CPP_ICAMERA_H
