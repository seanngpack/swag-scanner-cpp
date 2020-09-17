#ifndef SWAG_SCANNER_IMODEL_H
#define SWAG_SCANNER_IMODEL_H

#include <memory>
#include <vector>

namespace pcl {
    class PointXYZ;

    template<class pointT>
    class PointCloud;
}

namespace model {
    /**
     * Interface for scanning, calibration, and processing models.
     */
    class IModel {
    public:
        IModel() = default;

        /**
         * Clear stored pointclouds from memory.
         */
        void clear_clouds() {
            clouds.clear();
        }

        virtual ~IModel() {}

    protected:
        std::vector<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> clouds;

    };
}

#endif //SWAG_SCANNER_IMODEL_H
