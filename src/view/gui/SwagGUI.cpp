#include "SwagGUI.h"
#include "FormsPayload.h"
#include "MoveFormsPayload.h"
#include "SideBar.h"
#include "ScanControls.h"
#include "CalibrateControls.h"
#include "MoveControls.h"
#include "factory/ControllerFactory.h"
#include "ProcessControls.h"
#include "IControllerGUI.h"
#include "MoveControllerGUI.h"
#include "IFileHandler.h"
#include <QPushButton>
#include <QHBoxLayout>
#include <QDesktopWidget>
#include <QComboBox>
#include <QThreadPool>
#include <QPlainTextEdit>


SwagGUI::SwagGUI(controller::ControllerFactory *factory, QWidget *parent) :
        factory(factory),
        QMainWindow(parent) {
    thread_pool = QThreadPool::globalInstance();
    set_up_main();
    set_up_left();
    set_up_right();
}

// --------------------------------------------------------------------------------
//                          PUBLIC METHODS
// --------------------------------------------------------------------------------



// --------------------------------------------------------------------------------
//                          SLOTS
// --------------------------------------------------------------------------------


void SwagGUI::handle_scan_button_pressed(const FormsPayload &vars) {
    controller::IControllerGUI *c = factory->get_gui_controller("scan").get();
    c->update(vars);
    c->setAutoDelete(false);
    connect(c, SIGNAL(update_console(const std::string &)), this, SLOT(update_console(const std::string &)),
            Qt::UniqueConnection);
    thread_pool->start(c);
}

void SwagGUI::handle_calibrate_button_pressed(const FormsPayload &vars) {
    controller::IControllerGUI *c = factory->get_gui_controller("calibrate").get();
    c->update(vars);
    c->setAutoDelete(false);
    connect(c, SIGNAL(update_console(const std::string &)), this, SLOT(update_console(const std::string &)),
            Qt::UniqueConnection);
    thread_pool->start(c);
}

void SwagGUI::handle_process_button_pressed(const FormsPayload &vars) {
    controller::IControllerGUI *c = factory->get_gui_controller("process").get();
    c->update(vars);
    c->setAutoDelete(false);
    connect(c, SIGNAL(update_console(const std::string &)), this, SLOT(update_console(const std::string &)),
            Qt::UniqueConnection);
    thread_pool->start(c);
}

void SwagGUI::handle_move_button_pressed(const MoveFormsPayload &vars) {
    controller::IControllerGUI *c = factory->get_gui_controller("move").get();
    c->update(vars);
    c->setAutoDelete(false);
    connect(c, SIGNAL(update_console(const std::string &)), this, SLOT(update_console(const std::string &)),
            Qt::UniqueConnection);
    thread_pool->start(c);
    thread_pool->waitForDone();
}

void SwagGUI::handle_set_home_button_pressed() {
    controller::IControllerGUI *c = factory->get_gui_controller("move").get();
    // intentially casting because the alternative would be to use a HomeController class
    // where the run() method sets home. I think this is cleaner.
    // TODO: just use a home controller lol
    dynamic_cast<controller::MoveControllerGUI *>(c)->set_home();
    update_console("set current position to home");
}

void SwagGUI::handle_combo_index_changed(int index) {
    if (index == 0) {
        scan_controls->setVisible(true);
        calibrate_controls->setVisible(false);
        process_controls->setVisible(false);
        move_controls->setVisible(false);
    } else if (index == 1) {
        scan_controls->setVisible(false);
        calibrate_controls->setVisible(true);
        process_controls->setVisible(false);
        move_controls->setVisible(false);
    } else if (index == 2) {
        scan_controls->setVisible(false);
        calibrate_controls->setVisible(false);
        process_controls->setVisible(true);
        move_controls->setVisible(false);
    } else if (index == 3) {
        scan_controls->setVisible(false);
        calibrate_controls->setVisible(false);
        process_controls->setVisible(false);
        move_controls->setVisible(true);
    }
}

void SwagGUI::handle_scan_cal_combo_changed(int index) {
    if (index == 0) {
        std::vector<std::string> scans = file::IFileHandler::get_all_scans();
        emit update_scan_list(scans);
    }
    if (index == 1) {
        std::vector<std::string> cals = file::IFileHandler::get_all_calibrations();
        emit update_cal_list(cals);
    }
}

void SwagGUI::update_console(const std::string &info) {
    console_widget->appendPlainText(QString::fromStdString(info));
}

// --------------------------------------------------------------------------------
//                          PRIVATE HELPERS
// --------------------------------------------------------------------------------

void SwagGUI::set_up_main() {
    main = new QWidget;
    main_layout = new QHBoxLayout;
    main->setLayout(main_layout);
    setCentralWidget(main);

    left_side = new SideBar;
    right_side = new QVBoxLayout;

    // connect left and right side to main
    main_layout->addWidget(left_side, 1);
    main_layout->addLayout(right_side, 4);

    QDesktopWidget dw;
    setFixedSize(dw.width() * .4, dw.height() * .4);

}

void SwagGUI::set_up_left() {
    connect(left_side, SIGNAL(scan_cal_combo_changed(int)), this,
            SLOT(handle_scan_cal_combo_changed(int)));
    connect(this, SIGNAL(update_scan_list(const std::vector<std::string> &)), left_side,
            SLOT(update_scan_list(const std::vector<std::string> &)));
    connect(this, SIGNAL(update_cal_list(const std::vector<std::string> &)), left_side,
            SLOT(update_cal_list(const std::vector<std::string> &)));
    handle_scan_cal_combo_changed(0); // start the GUI displaying latest scans
}

void SwagGUI::set_up_right() {
    // edit right side
    console_widget = new QPlainTextEdit;
    scan_controls = new ScanControls;
    calibrate_controls = new CalibrateControls;
    process_controls = new ProcessControls;
    move_controls = new MoveControls;
    options_combo_box = new QComboBox;

    options_combo_box->addItem("scan");
    options_combo_box->addItem("calibrate");
    options_combo_box->addItem("process");
    options_combo_box->addItem("move");

    right_side->addWidget(options_combo_box);
    right_side->addWidget(scan_controls);
    right_side->addWidget(calibrate_controls);
    right_side->addWidget(process_controls);
    right_side->addWidget(move_controls);

    connect(options_combo_box, SIGNAL(currentIndexChanged(int)), this, SLOT(handle_combo_index_changed(int)));
    connect(scan_controls, SIGNAL(scan_button_pressed(const FormsPayload &)), this,
            SLOT(handle_scan_button_pressed(const FormsPayload &)));
    connect(calibrate_controls, SIGNAL(calibrate_button_pressed(const FormsPayload &)), this,
            SLOT(handle_calibrate_button_pressed(const FormsPayload &)));
    connect(process_controls, SIGNAL(process_button_pressed(const FormsPayload &)), this,
            SLOT(handle_process_button_pressed(const FormsPayload &)));
    connect(move_controls, SIGNAL(move_button_pressed(const MoveFormsPayload &)), this,
            SLOT(handle_move_button_pressed(const MoveFormsPayload &)));
    connect(move_controls, SIGNAL(set_home_button_pressed()), this,
            SLOT(handle_set_home_button_pressed()));


    calibrate_controls->setVisible(false);
    process_controls->setVisible(false);
    move_controls->setVisible(false);
//    right_side->addStretch();
    right_side->addWidget(console_widget);
}

SwagGUI::~SwagGUI() = default;


