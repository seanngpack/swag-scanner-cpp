#ifndef SWAG_SCANNER_CONTROLLERMANAGER_H
#define SWAG_SCANNER_CONTROLLERMANAGER_H

#include <boost/program_options.hpp>
#include <memory>

namespace controller {
    class IController;
    class IControllerGUI;
    class ControllerManagerCache;
}

class SwagGUI;

namespace controller {
    class ControllerManager {
    public:

        ControllerManager();

        ~ControllerManager();

        std::shared_ptr<IController> get_controller(const boost::program_options::variables_map &vm);

        std::shared_ptr<IController> get_controller(const std::string &name);

        std::shared_ptr<IControllerGUI> get_gui_controller(const std::string &name);

        std::shared_ptr<SwagGUI> get_gui();


    private:
        std::unique_ptr<ControllerManagerCache> cache;

    };
}

#endif //SWAG_SCANNER_CONTROLLERMANAGER_H
