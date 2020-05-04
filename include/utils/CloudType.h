#ifndef SWAG_SCANNER_CLOUDTYPE_H
#define SWAG_SCANNER_CLOUDTYPE_H

/**
 * Enumeration for point cloud states.
 * RAW = cloud that has had no processing done to it yet.
 * FILTERED = cloud has downsampling and point nuking done to it.
 * SEGMENTED = cloud that is both FILTERED and has its plane removed.
 * NORMAL = normal cloud.
 */
enum class CloudType {
    RAW,
    FILTERED,
    SEGMENTED,
    NORMAL
};

#endif //SWAG_SCANNER_CLOUDTYPE_H
