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
         * TBH, this doesn't really need to anything either, but need to override to conform to superclass.
         */
        void update(const IFormsPayload &payload) override;

    protected:
        std::shared_ptr<model::ProcessingModel> model;
        std::string cloud_path;
    };
}

#endif //SWAG_SCANNER_EDITINGCONTROLLERGUI_H
