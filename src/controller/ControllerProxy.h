#ifndef SWAG_SCANNER_CONTROLLERPROXY_H
#define SWAG_SCANNER_CONTROLLERPROXY_H

#include "IController.h"
#include "IControllerGUI.h"
#include <memory>
#include <unordered_map>
#include <string>

namespace controller {
    /**
     * ControllerProxy caches controllers or uses ControllerFactory to create a new one if it doesn't exist
     * in the cache.
     */
    class ControllerProxy {
    public:
        ControllerProxy();

        std::shared_ptr<IControllerGUI> get_gui_controller(const std::string &name);

        std::shared_ptr<IControllerGUI> get_controller(const std::string &name);


    private:
        std::unordered_map<std::string, IController> controller_cache;

        std::unordered_map<std::string, IControllerGUI> gui_controller_cache;
    };
}

#endif //SWAG_SCANNER_CONTROLLERPROXY_H
