#ifndef SWAG_SCANNER_ICONTROLLERGUI_H
#define SWAG_SCANNER_ICONTROLLERGUI_H

#include <memory>
#include <string>
#include "IController.h"

class SwagGUI;

namespace controller {
/**
 * Represents a controller for GUIs.
 */
    class IControllerGUI : public IController {
    public:

        virtual ~IControllerGUI() {}

        /**
         * Get the name from GUI.
         * @return name.
         */
        std::string get_name();

        /**
         * Get the degree from GUI.
         * @return degree.
         */
        int get_deg();

        /**
         * Get the rot from GUI.
         * @return rot.
         */
        int get_rot();

        /**
         * Write message to GUI console.
         */
        void update_console(const std::string &info);

    protected:
        std::shared_ptr<SwagGUI> gui;


    };

}

#endif //SWAG_SCANNER_ICONTROLLERGUI_H
