#ifndef SWAG_SCANNER_MOVECONTROLLERGUI_H
#define SWAG_SCANNER_MOVECONTROLLERGUI_H

#include "MoveController.h"
#include "IControllerGUI.h"

namespace arduino {
    class Arduino;
}

class SwagGUI;

class IFormsPayload;

namespace controller {
    class MoveControllerGUI : public MoveController, public IControllerGUI {
    public:
        MoveControllerGUI(std::shared_ptr<arduino::Arduino> arduino,
                          std::shared_ptr<SwagGUI> gui);

        /**
         * This run method is just like the base's run method, but it fetches move info before running.
         */
        void run() override;

        void update(const IFormsPayload &payload) override;

    private:

    };
}

#endif //SWAG_SCANNER_MOVECONTROLLERGUI_H
