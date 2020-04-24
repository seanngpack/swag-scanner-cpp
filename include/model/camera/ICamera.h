//
// Created by Sean ng pack on 4/24/20.
//

#ifndef SWAG_SCANNER_CPP_ICAMERA_H
#define SWAG_SCANNER_CPP_ICAMERA_H

#include <vector>
#include <librealsense2/rs.hpp>
#include "CameraTypes.h"

namespace camera {
    /**
     * Interface for a camera. Contains methods to get depth maps and return them as 1d vectors.
     */
    class ICamera {
    public:
        /**
         * Get the camera intrinsics and store them to class fields.
         * @return a ss_camera struct of the camera intrinsics.
         */
        virtual camera::ss_intrinsics getInstrinsics() = 0;

        /**
         * Take an image which is a 2d vector of depth values.
         * @return the depth map.
         */
        virtual rs2::depth_frame getDepthFrame() = 0;

        /**
         * TODO: may not be necessary, let's look at librealsense's return types
         * Flatten the 2d depth map and convert to meters if necessary.
         * @return a 1d vector of the depth values.
         */
        virtual std::vector<float> getDepthVector() = 0;

        /**
         * Virtual destructor, must be defined or else it will never call the base class's destructor.
         */
        ~ICamera();


    private:
        camera::ss_intrinsics intrinsics;

    };

}


#endif //SWAG_SCANNER_CPP_ICAMERA_H
