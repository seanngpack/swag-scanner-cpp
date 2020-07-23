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
         * Get copy of camera intrinsics.
         * @return a pointer to ss_camera struct of the camera intrinsics.
         */
        virtual camera::ss_intrinsics get_intrinsics() = 0;

        /**
         * Get copy of intrinsics after processing is applied. Only applies to realsense
         * sensors.
         * @return intrinsics after decimation changes the resolution of image.
         */
        virtual camera::ss_intrinsics get_intrinsics_processed() = 0;

        /**
         * Get depth image and set to class variable.
         */
        virtual void scan() = 0;

        /**
         * Get depth frame vector.
         * @return the depth map.
         */
        virtual std::vector<uint16_t> get_depth_frame() = 0;

        /**
         * Get the depth frame vector after filtering.
         * @return filtered depth frame vector.
         */
        virtual std::vector<uint16_t> get_depth_frame_processed() = 0;

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
