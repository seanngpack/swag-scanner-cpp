#ifndef SWAG_SCANNER_CLOUDTYPE_H
#define SWAG_SCANNER_CLOUDTYPE_H

#include <string>

/**
 * Enumeration for point calibration states.
 * RAW = calibration that has had no processing done to it yet.
 * FILTERED = cloud that has bilateral filtering, nan points removed, and point cropping done to it.
 * REGISTERED = registered clouds.
 * NORMAL = normal clouds.
 * CALIBRATION = calibration clouds.
 */
namespace CloudType {
    enum class Type {
        NONE,
        RAW,
        FILTERED,
//        PROCESSED,
        REGISTERED,
//        NORMAL,
        CALIBRATION
    };

    static const Type All[] = {CloudType::Type::RAW,
                               CloudType::Type::FILTERED,
//                               CloudType::Type::PROCESSED,
                               CloudType::Type::REGISTERED,
//                               CloudType::Type::NORMAL,
                               CloudType::Type::CALIBRATION};

    inline std::string String(CloudType::Type type) {
        switch (type) {
            case CloudType::Type::RAW:
                return "raw";
            case CloudType::Type::FILTERED:
                return "filtered";
//            case CloudType::Type::PROCESSED:
//                return "processed";
//            case CloudType::Type::NORMAL:
//                return "normal";
            case CloudType::Type::REGISTERED:
                return "registered";
            case CloudType::Type::CALIBRATION:
                return "calibration";
            default:
                return "error, enum not defined for cloudtype";
        }
    }
}

#endif //SWAG_SCANNER_CLOUDTYPE_H
