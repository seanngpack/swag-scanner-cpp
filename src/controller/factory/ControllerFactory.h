#ifndef SWAG_SCANNER_CONTROLLERFACTORY_H
#define SWAG_SCANNER_CONTROLLERFACTORY_H

#include <boost/program_options.hpp>
#include <memory>

namespace controller {
    class IController;
    class IControllerGUI;
    class ControllerFactoryCache;
}

namespace controller {
    class ControllerFactory {
    public:

        ControllerFactory();

        ~ControllerFactory();

        std::shared_ptr<IController> get_controller(const boost::program_options::variables_map &vm);

        std::shared_ptr<IController> get_controller(const std::string &name);

        std::shared_ptr<IControllerGUI> get_gui_controller(const std::string &name);


    private:
        std::unique_ptr<ControllerFactoryCache> cache;

    };
}

#endif //SWAG_SCANNER_CONTROLLERFACTORY_H
