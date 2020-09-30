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

        /**
         * Yes, I'm using string comparison because the input is from the CLI, so it will be a string anyways.
         */
        std::shared_ptr<IController> get_controller(const std::string &name);

        /**
         * This on the other hand, I can use an enum for later.
         */
        std::shared_ptr<IControllerGUI> get_gui_controller(const std::string &name);

        std::shared_ptr<SwagGUI> get_gui();


    private:
        std::unique_ptr<ControllerManagerCache> cache;

    };
}

#endif //SWAG_SCANNER_CONTROLLERMANAGER_H
