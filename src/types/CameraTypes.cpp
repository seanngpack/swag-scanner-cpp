#include "CameraTypes.h"

camera::intrinsics::intrinsics(rs2_intrinsics intrin, float depth_scale) :
        width(intrin.width), height(intrin.height), fx(intrin.fx), fy(intrin.fy),
        ppx(intrin.ppx), ppy(intrin.ppy), model(intrin.model),
        depth_scale(depth_scale) {
    for (int i = 0; i < 5; i++) {
        coeffs[i] = intrin.coeffs[i];
    }
}

camera::intrinsics::intrinsics(int width,
                               int height,
                               float fx,
                               float fy,
                               float ppx,
                               float ppy,
                               rs2_distortion model,
                               float coeffs[5],
                               float depth_scale) : width(width), height(height), fx(fx),
                                                          fy(fy), ppx(ppx), ppy(ppy), model(model),
                                                          depth_scale(depth_scale) {
    for (int i = 0; i < 5; i++) {
        this->coeffs[i] = coeffs[i];
    }
}

std::string camera::intrinsics::to_string() const {
    return "width: " + std::to_string(width) + "\n" +
           "height: " + std::to_string(height) + "\n" +
           "fx: " + std::to_string(fx) + "\n" +
           "fy: " + std::to_string(fy) + "\n" +
           "ppx: " + std::to_string(ppx) + "\n" +
           "ppy: " + std::to_string(ppy) + "\n" +
           "depth scale: " + std::to_string(depth_scale);
}

camera::intrinsics::~intrinsics() {}



