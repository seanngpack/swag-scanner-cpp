#ifndef SWAG_SCANNER_CLOUDTYPE_H
#define SWAG_SCANNER_CLOUDTYPE_H

/**
 * Enumeration for point cloud states.
 * RAW = cloud that has had no processing done to it yet.
 * FILTERED = cloud has downsampling and point nuking done to it.
 * SEGMENTED = cloud that is both FILTERED and has its plane removed.
 * NORMAL = normal cloud.
 */
namespace CloudType {
    enum class Type {
        RAW,
        FILTERED,
        SEGMENTED,
        NORMAL,
        CALIBRATION
    };

    static const Type All[] = {CloudType::Type::RAW,
                               CloudType::Type::FILTERED,
                               CloudType::Type::SEGMENTED,
                               CloudType::Type::NORMAL,
                               CloudType::Type::CALIBRATION};

    inline std::string String(CloudType::Type type) {
        switch (type) {
            case CloudType::Type::RAW:
                return "raw";
            case CloudType::Type::FILTERED:
                return "filtered";
            case CloudType::Type::SEGMENTED:
                return "segmented";
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
