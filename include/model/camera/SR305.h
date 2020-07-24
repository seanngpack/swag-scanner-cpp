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

        camera::ss_intrinsics get_intrinsics_processed() override;

        void scan() override;

        std::vector<uint16_t> get_depth_frame() override;

        /**
         * Get the processed depth frame. Subsampling and spatial filtering applied.
         * @return processed depth frame.
         */
        std::vector<uint16_t> get_depth_frame_processed() override;

        /**
         * Divide resolution by magnitude.
         * @param mag [2 - 8] default = 2
         */
        virtual void set_decimation_magnitude(int mag);

        /**
         * # of filter iterations;
         * @param mag [1 - 5] default = 2;
         */
        virtual void set_spatial_filter_magnitude(int mag);

        /**
         * Alpha factor in exponential moving average.
         * alpha = 1, no filter
         * alpha = 0, infinite filter
         * lower = more aggressive smoothing, edges become more rounded.
         * @param a [.25 - 1], default = .5
         */
        virtual void set_spatial_smooth_alpha(float a);

        /**
         * Step size boundary, establishes threshold used to preserve "edges".
         *  If the depth value between neighboring pixels exceed the depth threshold set by this delta parameter,
         *  then alpha will be temporarily reset to 1 (no filtering). This basically means that if an edge is observed,
         *  then the smoothing is temporarily turned off.
         * @param d [1 - 50], default = 20
         */
        virtual void set_spatial_smooth_delta(int d);

        ~SR305();


    private:
        rs2::device dev;
        rs2::pipeline pipe;
        rs2::pipeline_profile pipe_profile;
        rs2::decimation_filter dec_filter;
        rs2::spatial_filter spat_filter;
        rs2::hole_filling_filter hole_filter;
        rs2::frame current_frame;
        float depth_scale;

        // decimation filter parameters
        int decimation_magnitude = 2;

        // spatial edge-preservation filter parameters
        int spatial_filter_magnitude = 1;
        float spatial_smooth_alpha = 0.45;
        int spatial_smooth_delta = 5;


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
