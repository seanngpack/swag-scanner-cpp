#include <SR305.h>

camera::SR305::SR305() {
    initialize_camera();
}

camera::ss_intrinsics *camera::SR305::get_intrinsics() {
    return &intrinsics;
}

std::vector<uint16_t> camera::SR305::get_depth_frame() {
    rs2::frameset frames = pipe.wait_for_frames();
    rs2::frame frame = frames.first(RS2_STREAM_DEPTH);
    const uint16_t *depth_frame_arr;
    if (frame) {
        const auto *arr = static_cast<const uint16_t *>(frame.get_data());
        std::vector<uint16_t> depth_frame(arr, arr + (width * height));
        return depth_frame;
    }

    throw std::runtime_error("Cannot grab depth frame from video stream, something"
                             "is horribly wrong.");
}


void camera::SR305::initialize_camera() {
    // grab the context and set the device
    rs2::context ctx;
    auto device_list = ctx.query_devices();
    if (device_list.size() == 0) {
        throw std::runtime_error("No device detected. Make sure your camera is plugged in");
    }
    dev = device_list.front();

    // configure the pipeline
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH);

    // start the pipeline
    rs2::pipeline p;
    pipe = p;
    pipe_profile = p.start(cfg);

    // grab the depth scale
    auto sensor = pipe_profile.get_device().first<rs2::depth_sensor>();


    // grab the intrinsics
    auto intrin = pipe_profile.get_stream(RS2_STREAM_DEPTH)
            .as<rs2::video_stream_profile>().get_intrinsics();
    intrinsics = camera::ss_intrinsics(
            intrin.width,
            intrin.height,
            intrin.fx,
            intrin.fy,
            intrin.ppx,
            intrin.ppy,
            intrin.model,
            intrin.coeffs,
            sensor.get_depth_scale());
}


camera::SR305::~SR305() {
    std::cout << "calling SR305 destructor \n";;
}

