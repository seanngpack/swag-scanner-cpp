#ifndef SWAG_SCANNER_CPP_ICAMERA_H
#define SWAG_SCANNER_CPP_ICAMERA_H

#include "CameraTypes.h"
#include <vector>
#include <memory>
#include <librealsense2/rs.hpp>

namespace pcl {
    class PointXYZ;

    template<class pointT>
    class PointCloud;
}

namespace camera {

    /**
     * Interface for a camera. Contains methods to get depth maps and return them as 1d vectors.
     */
    class ICamera {
    public:

        /**
         * Get copy of camera intrin.
         * @return a pointer to ss_camera struct of the camera intrin.
         */
        virtual camera::intrinsics get_intrinsics() = 0;

        /**
         * Get copy of intrin after processing is applied. Only applies to realsense
         * sensors.
         * @return intrin after decimation changes the resolution of image.
         */
        virtual camera::intrinsics get_intrinsics_processed() = 0;

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
         * Create new pointcloud given depth frame and intrinsics.
         */
        virtual std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>
        create_point_cloud(const std::vector<uint16_t> &depth_frame,
                           const camera::intrinsics &intrinsics) = 0;

        /**
         * Virtual destructor, must be defined or else it will never call the base class's destructor.
         */
        virtual ~ICamera() {}


    protected:
        intrinsics intrin;
    };

}


#endif //SWAG_SCANNER_CPP_ICAMERA_H
