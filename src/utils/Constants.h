#ifndef SWAG_SCANNER_CONSTANTS_H
#define SWAG_SCANNER_CONSTANTS_H

namespace constants {
    inline constexpr float BED_DIAMETER = .180;
    inline constexpr float MAX_SCAN_HEIGHT = .3;
    inline constexpr int SENSOR_ANGLE = 45;
    inline constexpr float CENTER_TO_SENSOR_H = .180;
    inline constexpr float CENTER_TO_SENSOR_V = .180;

    // crop coefficients for calibration
    // can make them a bit bigger because of robust calibration plane detection
    inline constexpr float cal_min_x = -.10;
    inline constexpr float cal_max_x = .10;
    inline constexpr float cal_min_y = - 100;
    inline constexpr float cal_max_y = .13;
    inline constexpr float cal_min_z = -100;
    inline constexpr float cal_max_z = .49;

    // dimensions of scanning box
    // crop coefficients after calibration is done
    inline constexpr float scan_min_x = -.11;
    inline constexpr float scan_max_x = .11;
    inline constexpr float scan_min_y = - .11;
    inline constexpr float scan_max_y = .11;
    inline constexpr float scan_min_z = 0;
    inline constexpr float scan_max_z = .17;

}

#endif //SWAG_SCANNER_CONSTANTS_H
