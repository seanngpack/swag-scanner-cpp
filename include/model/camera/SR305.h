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

        camera::ss_intrinsics get_intrinsics() override;

        camera::ss_intrinsics get_intrinsics_processed();

        void scan() override;

        std::vector<uint16_t> get_depth_frame() override;

        /**
         * Get the processed depth frame. Subsampling, spatial filtering, temporal filtering applied.
         * @return processed depth frame.
         */
        std::vector<uint16_t> get_depth_frame_processed() override;

        virtual void set_decimation_magnitude(int mag);

        virtual void set_spatial_filter_magnitude(int mag);

        virtual void set_spatial_smooth_alpha(float a);

        virtual void set_spatial_smooth_delta(int d);

        virtual void set_temporal_smooth_alpha(float a);

        virtual void set_temporal_smooth_delta(float d);

        virtual void set_temporal_persistency_idx(int i);

        ~SR305();


    private:
        rs2::device dev;
        rs2::pipeline pipe;
        rs2::pipeline_profile pipe_profile;
        rs2::decimation_filter dec_filter;
        rs2::spatial_filter spat_filter;
        rs2::temporal_filter temp_filter;
        rs2::frame current_frame;
        float depth_scale;

        // decimation filter parameters
        int decimation_magnitude = 2;

        // spatial edge-preservation filter parameters
        int spatial_filter_magnitude = 2;
        float spatial_smooth_alpha = 0.5;
        int spatial_smooth_delta = 20;

        // temporal filter parameters
        float temporal_smooth_alpha = .4;
        int temporal_smooth_delta = 20;
        float temporal_persistency_idx = 3.0;


        int width = 640;
        int height = 480;

        /**
         * Initialize the pipeline and grab camera parameters for class fields.
         */
        void initialize_camera();

        /**
         * Get depth frame.
         * @return rs2 frame.
         */
        rs2::frame get_rs2_frame();


    };


}

#endif //SWAG_SCANNER_SR305_H
