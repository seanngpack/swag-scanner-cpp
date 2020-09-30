#ifndef SWAG_SCANNER_EDITINGCONTROLLERGUI_H
#define SWAG_SCANNER_EDITINGCONTROLLERGUI_H

#include "IController.h"
#include "IControllerGUI.h"

namespace model {
    class ProcessingModel;
}

namespace pcl {
    class PointXYZ;

    template<class pointT>
    class PointCloud;
}

namespace controller {

    class EditingControllerGUI : public IControllerGUI {
    public:
        EditingControllerGUI(std::shared_ptr<model::ProcessingModel> model,
                             std::shared_ptr<SwagGUI> gui);

        /**
         * Display the cloud on the screen.
         *
         * Note: current only displays the registered cloud
         */
        void run() override;

        /**
         * Set the path of the cloud to load.
         *
         * Note: Right now this method does nothing
         *
         * @param path path of the cloud to load.
         */
        void set_cloud_path(const std::string &path);

    protected:
        std::shared_ptr<model::ProcessingModel> model;
        std::string cloud_path;
    };
}

#endif //SWAG_SCANNER_EDITINGCONTROLLERGUI_H
