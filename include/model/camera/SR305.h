#ifndef SWAG_SCANNER_SR305_H
#define SWAG_SCANNER_SR305_H

#include "ICamera.h"

namespace camera {

    /**
     * SR305 camera implementation. Belongs to intel realsense family of depth-sensing cameras.
     */
    class SR305 : public ICamera {

    public:
        SR305();

        camera::ss_intrinsics *get_intrinsics() override;

        std::vector<uint16_t> get_depth_frame() override;

        ~SR305();


    private:
        rs2::device dev;
        rs2::pipeline pipe;
        rs2::pipeline_profile pipe_profile;

        int width = 640;
        int height = 480;

        /**
         * Initialize the pipeline and grab camera parameters for class fields.
         */
        void initialize_camera();



    };


}

#endif //SWAG_SCANNER_SR305_H
