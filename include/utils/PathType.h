#ifndef SWAG_SCANNER_PATHTYPE_H
#define SWAG_SCANNER_PATHTYPE_H

#include <iostream>

/**
 * Enumeration for point cloud states.
 * RAW = cloud that has had no processing done to it yet.
 * FILTERED = cloud has downsampling and point nuking done to it.
 * SEGMENTED = cloud that is both FILTERED and has its plane removed.
 * NORMAL = normal cloud.
 */
namespace PathType {
    enum class Type {
        ALL_DATA_FOLDER,
        SCAN_FOLDER
    };

    static const Type All[] = {PathType::Type::ALL_DATA_FOLDER,
                               PathType::Type::SCAN_FOLDER
    };

    inline std::string String(PathType::Type type) {
        switch (type) {
            case PathType::Type::ALL_DATA_FOLDER:
                return "all_data_folder";
            case PathType::Type::SCAN_FOLDER:
                return "scan_folder";
            default:
                return "error, enum not defined for pathtype";
        }
    }
}
#endif //SWAG_SCANNER_PATHTYPE_H
