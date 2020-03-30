#include <vector>
#include <librealsense2/rs.hpp>

/**
 * Interface for a camera. Contains methods to get depth maps and return them as 1d vectors.
 */
class ICamera {
public:
    /**
     * Get the camera intrinsics and store them to class fields.
     * @return a rs2_intrinsics struct of the camera intrinsics.
     */
    virtual rs2_intrinsics getInstrinsics() = 0;

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
};
