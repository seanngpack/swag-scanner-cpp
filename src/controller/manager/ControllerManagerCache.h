#ifndef SWAG_SCANNER_CONTROLLERMANAGERCACHE_H
#define SWAG_SCANNER_CONTROLLERMANAGERCACHE_H

#include <memory>
#include <boost/program_options.hpp>

namespace camera {
    class SR305;
}

namespace arduino {
    class Arduino;
}

namespace model {
    class CalibrationModel;

    class ScanModel;

    class ProcessingModel;
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

    class ControllerManager;
}


namespace controller {
    /**
     * This manages objects needed to create controllers.
     */
    class ControllerManagerCache {
    public:
        /**
         * Default constructor preallocates expensive objects that are probably gonna be used.
         * Preallocated objects: Model, ScanFileHandler, CalibrationFileHandler
         */
        ControllerManagerCache(ControllerManager *factory);

        ~ControllerManagerCache();

        // --------------------------------------------------------------------------------
        //                          Get objects
        // --------------------------------------------------------------------------------

        std::shared_ptr<camera::SR305> get_camera();

        std::shared_ptr<arduino::Arduino> get_arduino();

        std::shared_ptr<model::CalibrationModel> get_calibration_model();

        std::shared_ptr<model::ScanModel> get_scan_model();

        std::shared_ptr<model::ProcessingModel> get_processing_model();

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
        ControllerManager *factory;

        std::shared_ptr<camera::SR305> camera;
        std::shared_ptr<arduino::Arduino> arduino;
        std::shared_ptr<model::CalibrationModel> calibration_model;
        std::shared_ptr<model::ScanModel> scan_model;
        std::shared_ptr<model::ProcessingModel> processing_model;

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

#endif //SWAG_SCANNER_CONTROLLERMANAGERCACHE_H
