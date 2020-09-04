#ifndef SWAG_SCANNER_CONTROLLERFACTORY_H
#define SWAG_SCANNER_CONTROLLERFACTORY_H

#include "ControllerFactoryCache.h"
#include <boost/program_options.hpp>

namespace controller {
    class IController;
}

namespace controller {
    class ControllerFactory {
    public:

        ControllerFactory();

        std::shared_ptr<IController> get_controller(const boost::program_options::variables_map &vm);

        std::shared_ptr<IController> get_controller(const std::string &name);


    private:
        std::shared_ptr<ControllerFactoryCache> cache;

    };
}

#endif //SWAG_SCANNER_CONTROLLERFACTORY_H
