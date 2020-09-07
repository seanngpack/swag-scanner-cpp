#ifndef SWAG_SCANNER_ICONTROLLERGUI_H
#define SWAG_SCANNER_ICONTROLLERGUI_H

#include <memory>
#include <string>
#include "IController.h"

class SwagGUI;

class IFormsPayload;

namespace controller {
    /**
     * Represents a controller for GUIs.
     */
    class IControllerGUI : public IController {
    public:

        IControllerGUI() = default;

        virtual ~IControllerGUI() = default;

        /**
         * Update the controller with the given payload.
         *
         * @param payload payload from GUI.
         */
        virtual void update(const IFormsPayload &payload) = 0;

        /**
         * Write message to GUI console.
         */
        virtual void update_console(const std::string &info) = 0;

        std::vector<std::string> get_all_scans();

        std::vector<std::string> get_all_calibrations();

    protected:
        //TODO: later make this a variable and add a constructor.
//        std::shared_ptr<SwagGUI> gui;


    };

}

#endif //SWAG_SCANNER_ICONTROLLERGUI_H
