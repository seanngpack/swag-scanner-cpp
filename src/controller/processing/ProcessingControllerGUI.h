#ifndef SWAG_SCANNER_PROCESSINGCONTROLLERGUI_H
#define SWAG_SCANNER_PROCESSINGCONTROLLERGUI_H

#include "ProcessingController.h"
#include "IControllerGUI.h"

namespace controller {
    class ProcessingControllerGUI : public ProcessingController, public IControllerGUI {
    public:
        ProcessingControllerGUI(std::shared_ptr<model::ProcessingModel> model,
                                std::shared_ptr<SwagGUI> gui);

        /**
         * This run method is just like the base's run method, but it fetches move info before running.
         */
        void run() override;

        void update(const IFormsPayload &payload) override;
    };


}

#endif //SWAG_SCANNER_PROCESSINGCONTROLLERGUI_H
