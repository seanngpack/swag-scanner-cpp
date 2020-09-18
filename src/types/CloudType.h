#ifndef SWAG_SCANNER_CLOUDTYPE_H
#define SWAG_SCANNER_CLOUDTYPE_H

#include <string>

/**
 * Enumeration for point calibration states.
 * RAW = calibration that has had no processing done to it yet.
 * FILTERED = calibration has downsampling and point nuking done to it.
 * SEGMENTED = calibration that is both FILTERED and has its plane removed.
 * NORMAL = normal calibration.
 */
namespace CloudType {
    enum class Type {
        NONE,
        RAW,
        FILTERED,
        PROCESSED,
        NORMAL,
        CALIBRATION
    };

    static const Type All[] = {CloudType::Type::RAW,
                               CloudType::Type::FILTERED,
                               CloudType::Type::PROCESSED,
                               CloudType::Type::NORMAL,
                               CloudType::Type::CALIBRATION};

    inline std::string String(CloudType::Type type) {
        switch (type) {
            case CloudType::Type::RAW:
                return "raw";
            case CloudType::Type::FILTERED:
                return "filtered";
            case CloudType::Type::PROCESSED:
                return "processed";
            case CloudType::Type::NORMAL:
                return "normal";
            case CloudType::Type::CALIBRATION:
                return "calibration";
            default:
                return "error, enum not defined for cloudtype";
        }
    }
}

#endif //SWAG_SCANNER_CLOUDTYPE_H
