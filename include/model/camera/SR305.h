#ifndef SWAG_SCANNER_SR305_H
#define SWAG_SCANNER_SR305_H

#include "ICamera.h"
#include "CameraTypes.h"

namespace camera {

    /**
     * SR305 camera implementation. Belongs to intel realsense family of depth-sensing cameras.
     */
    class SR305 : public ICamera {

    public:
        SR305();

        camera::ss_intrinsics get_instrinsics() override;

        rs2::depth_frame get_depth_frame() override;

        std::vector<float> get_depth_vector() override;

        ~SR305();

    private:
        /**
         * Create a realsense context object
         */
        void initialize_camera();
    };


}

#endif //SWAG_SCANNER_SR305_H
