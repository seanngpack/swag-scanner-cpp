#ifndef SWAG_SCANNER_CONTROLLERFACTORY_H
#define SWAG_SCANNER_CONTROLLERFACTORY_H

#include "ControllerFactoryCache.h"
#include <boost/program_options.hpp>
#include <SR305.h>
#include <Arduino.h>
#include <Model.h>
#include <Visualizer.h>
#include <ScanFileHandler.h>
#include <CalibrationFileHandler.h>

namespace controller {
    class IController;
}

namespace controller {
    class ControllerFactory {
    public:

        ControllerFactory() = default;

        std::unique_ptr<IController> create(boost::program_options::variables_map vm);


    private:
        std::unique_ptr<ControllerFactoryCache> cache;

        /**
         * Creates a new scanning controller. if -name is not passed, it will create
         * a new folder.
         * @param vm
         * @return
         */
        std::unique_ptr<IController>
        create_scan_controller(boost::program_options::variables_map vm);

        /**
         * Creates a new calibration controller. If -name is not passed then it will
         * use the latest calibration folder.
         * @param vm
         * @return
         */
        std::unique_ptr<IController>
        create_calibrate_controller(boost::program_options::variables_map vm);

        /**
         * Create a processing controller. If -name is not passed then it will use the latest scan according
         * to the info.json file.
         * @param vm
         * @return
         */
        std::unique_ptr<IController>
        create_processing_controller(boost::program_options::variables_map vm);

        /**
         * Create a filter testing controller.
         * @param vm
         * @return
         */
        std::unique_ptr<IController>
        create_filter_testing_controller(boost::program_options::variables_map vm);

        /**
         * Create a move controller.
         * @param vm
         * @return
         */
        std::unique_ptr<IController>
        create_move_controller(boost::program_options::variables_map vm);

        /**
         * Create a home controller to set home position.
         * @param vm
         * @return
         */
        std::unique_ptr<IController>
        create_home_controller(boost::program_options::variables_map vm);
    };
}

#endif //SWAG_SCANNER_CONTROLLERFACTORY_H
