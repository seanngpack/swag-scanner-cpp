#ifndef SWAG_SCANNER_CONTROLLERFACTORYCACHE_H
#define SWAG_SCANNER_CONTROLLERFACTORYCACHE_H

#include <memory>
#include <boost/program_options.hpp>

namespace camera {
    class SR305;
}

namespace arduino {
    class Arduino;
}

namespace model {
    class Model;
}

namespace visual {
    class Visualizer;
}

namespace file {
    class CalibrationFileHandler;

    class ScanFileHandler;
}

class SwagGUI;

namespace controller {
    class ScanController;

    class ScanControllerGUI;

    class CalibrationController;

    class CalibrationControllerGUI;

    class ProcessingController;

    class ProcessingControllerGUI;

    class FilterTestingController;

    class HomeController;

    class MoveController;

    class MoveControllerGUI;

    class ControllerFactory;
}


namespace controller {
    /**
     * This manages objects needed to create controllers.
     */
    class ControllerFactoryCache {
    public:
        /**
         * Default constructor preallocates expensive objects that are probably gonna be used.
         * Preallocated objects: Model, ScanFileHandler, CalibrationFileHandler
         */
        ControllerFactoryCache(ControllerFactory *factory);

        ~ControllerFactoryCache();

        // --------------------------------------------------------------------------------
        //                          Get objects
        // --------------------------------------------------------------------------------

        std::shared_ptr<camera::SR305> get_camera();

        std::shared_ptr<arduino::Arduino> get_arduino();

        std::shared_ptr<model::Model> get_model();

        std::shared_ptr<visual::Visualizer> get_viewer();

        std::shared_ptr<file::ScanFileHandler> get_scan_file_handler();

        std::shared_ptr<file::CalibrationFileHandler> get_calibration_file_handler();

        std::shared_ptr<SwagGUI> get_gui();

        // --------------------------------------------------------------------------------
        //                          Get controllers
        // --------------------------------------------------------------------------------

        std::shared_ptr<controller::ScanController>
        get_scan_controller(const boost::program_options::variables_map &vm);

        std::shared_ptr<controller::ScanController> get_scan_controller();

        std::shared_ptr<controller::ScanControllerGUI> get_scan_controller_gui();


        std::shared_ptr<controller::CalibrationController>
        get_calibration_controller(const boost::program_options::variables_map &vm);

        std::shared_ptr<controller::CalibrationController> get_calibration_controller();

        std::shared_ptr<controller::CalibrationControllerGUI> get_calibration_controller_gui();

        std::shared_ptr<controller::ProcessingController>
        get_process_controller(const boost::program_options::variables_map &vm);

        std::shared_ptr<controller::ProcessingController> get_process_controller();

        std::shared_ptr<controller::ProcessingControllerGUI> get_process_controller_gui();

        std::shared_ptr<controller::MoveController>
        get_move_controller(const boost::program_options::variables_map &vm);

        std::shared_ptr<controller::MoveControllerGUI> get_move_controller_gui();

        std::shared_ptr<controller::HomeController>
        get_home_controller(const boost::program_options::variables_map &vm);

    private:
        ControllerFactory *factory;

        std::shared_ptr<camera::SR305> camera;
        std::shared_ptr<arduino::Arduino> arduino;
        std::shared_ptr<model::Model> model;
        std::shared_ptr<visual::Visualizer> viewer;
        std::shared_ptr<file::ScanFileHandler> scan_file_handler;
        std::shared_ptr<file::CalibrationFileHandler> calibration_file_handler;

        // controllers
        std::shared_ptr<controller::ScanController> scan_controller;
        std::shared_ptr<controller::CalibrationController> calibration_controller;
        std::shared_ptr<controller::ProcessingController> process_controller;
        std::shared_ptr<controller::MoveController> move_controller;

        //gui
        std::shared_ptr<SwagGUI> gui;

        // gui controllers
        std::shared_ptr<controller::ScanControllerGUI> scan_controller_gui;
        std::shared_ptr<controller::CalibrationControllerGUI> calibration_controller_gui;
        std::shared_ptr<controller::MoveControllerGUI> move_controller_gui;
        std::shared_ptr<controller::ProcessingControllerGUI> process_controller_gui;


    };
}

#endif //SWAG_SCANNER_CONTROLLERFACTORYCACHE_H
