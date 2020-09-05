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
        void update_name();

        /**
         * Get the degree from GUI.
         * @return degree.
         */
        void update_deg();

        /**
         * Get the rot from GUI.
         * @return rot.
         */
        void update_rot();

        /**
         * Write message to GUI console.
         */
        void update_console(const std::string &info);

        std::vector<std::string> get_all_scans();

        std::vector<std::string> get_all_calibrations();

    protected:
//        std::shared_ptr<SwagGUI> gui;


    };

}

#endif //SWAG_SCANNER_ICONTROLLERGUI_H
