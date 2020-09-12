#ifndef SWAG_SCANNER_CONSTANTS_H
#define SWAG_SCANNER_CONSTANTS_H

namespace constants {
    inline constexpr float BED_DIAMETER = .180;
    inline constexpr float MAX_SCAN_HEIGHT = .3;
    inline constexpr int SENSOR_ANGLE = 45;
    inline constexpr float CENTER_TO_SENSOR_H = .180;
    inline constexpr float CENTER_TO_SENSOR_V = .180;

    inline constexpr float min_x = -.10;
    inline constexpr float max_x = .10;
    inline constexpr float min_y = - 100;
    inline constexpr float max_y = .1;
    inline constexpr float min_z = -100;
    inline constexpr float max_z = .49;
}

#endif //SWAG_SCANNER_CONSTANTS_H
