#include <CameraTypes.h>

camera::ss_intrinsics::ss_intrinsics(rs2_intrinsics intrin, float depth_scale) :
        width(intrin.width), height(intrin.height), fx(intrin.fx), fy(intrin.fy),
        ppx(intrin.ppx), ppy(intrin.ppy), model(intrin.model), coeffs(intrin.coeffs),
        depth_scale(depth_scale) {}

camera::ss_intrinsics::ss_intrinsics(int width,
                                     int height,
                                     float fx,
                                     float fy,
                                     float ppx,
                                     float ppy,
                                     rs2_distortion model,
                                     float *coeffs,
                                     float depth_scale) : width(width), height(height), fx(fx),
                                                          fy(fy), ppx(ppx), ppy(ppy), model(model),
                                                          coeffs(coeffs), depth_scale(depth_scale) {}

std::string camera::ss_intrinsics::to_string() const {
    return "width: " + std::to_string(width) + "\n" +
           "height: " + std::to_string(height) + "\n" +
           "fx: " + std::to_string(fx) + "\n" +
           "fy: " + std::to_string(fy) + "\n" +
           "ppx: " + std::to_string(ppx) + "\n" +
           "ppy: " + std::to_string(ppy) + "\n" +
           "depth scale: " + std::to_string(depth_scale);
}

camera::ss_intrinsics::~ss_intrinsics() {}



