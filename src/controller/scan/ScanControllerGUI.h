#ifndef SWAG_SCANNER_SCANCONTROLLERGUI_H
#define SWAG_SCANNER_SCANCONTROLLERGUI_H

#include "ScanController.h"
#include "IControllerGUI.h"


namespace controller {
    class ScanControllerGUI : public ScanController, public IControllerGUI {
    public:
        ScanControllerGUI(std::shared_ptr<camera::ICamera> camera,
                          std::shared_ptr<arduino::Arduino> arduino,
                          std::shared_ptr<model::ScanModel> model,
                          std::shared_ptr<SwagGUI> gui);

        /**
         * This run method is just like the base's run method, but it fetches move info before running.
         */
        void run() override;

        void update(const IFormsPayload &payload) override;
    };
}

#endif //SWAG_SCANNER_SCANCONTROLLERGUI_H
