#include "SwagGUI.h"
#include "FormsPayload.h"
#include "SideBar.h"
#include "ScanControls.h"
#include "CalibrateControls.h"
#include "ControllerFactory.h"
#include "ProcessControls.h"
#include "IControllerGUI.h"
#include "CalibrationControllerGUI.h"
#include <QPushButton>
#include <QHBoxLayout>
#include <QDesktopWidget>
#include <QComboBox>
#include <QPlainTextEdit>
#include <QSpacerItem>
#include <iostream>


SwagGUI::SwagGUI(controller::ControllerFactory *factory, QWidget *parent) :
        factory(factory), QMainWindow(parent) {

    set_up_main();
    std::cout << "madeit here" << std::endl;
    set_up_left();
    std::cout << "madeit past left" << std::endl;
    set_up_right();
    std::cout << "madeit past right" << std::endl;
    std::cout << "madeit past controller" << std::endl;
}

void SwagGUI::set_controller(controller::IControllerGUI *c) {
    controller = c;
}

std::string SwagGUI::update_name() const {
    std::cout << "update name called " << std::endl;
    return name;
}

int SwagGUI::update_deg() const {
    return deg;
}

int SwagGUI::update_rot() const {
    return deg;
}

void SwagGUI::update_console(const std::string &info) {
    console_widget->appendPlainText(QString::fromStdString(info));
}

void SwagGUI::handle_combo_index_changed(int index) {
    if (index == 0) {
        scan_controls->setVisible(true);
        calibrate_controls->setVisible(false);
        process_controls->setVisible(false);
    } else if (index == 1) {
        scan_controls->setVisible(false);
        calibrate_controls->setVisible(true);
        process_controls->setVisible(false);
    } else if (index == 2) {
        scan_controls->setVisible(false);
        calibrate_controls->setVisible(false);
        process_controls->setVisible(true);
    }
}

void SwagGUI::handle_scan_button_pressed(const FormsPayload &vars) {
    name = vars.name;
    deg = vars.deg;
    rot = vars.rot;

    std::cout << "scan name: " << name << std::endl;
    std::cout << "scan deg: " << deg << std::endl;
    std::cout << "scan rot: " << rot << std::endl;
    // call observer to run controller method
}

void SwagGUI::handle_calibrate_button_pressed(const FormsPayload &vars) {
    name = vars.name;
    std::cout << "cal name: " << name << std::endl;
    deg = vars.deg;
    rot = vars.rot;
    std::cout << "bout to run calibration" << std::endl;
    controller->run();
    std::cout << "cal name: " << name << std::endl;
    std::cout << "cal deg: " << deg << std::endl;
    std::cout << "cal rot: " << rot << std::endl;
}

void SwagGUI::handle_process_button_pressed(const FormsPayload &vars) {
    name = vars.name;
    std::cout << "process name: " << name << std::endl;
    // call observer to run controller method
}

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
    connect(this, SIGNAL(update_scan_list(const std::vector<std::string> &)), left_side,
            SLOT(update_scan_list(const std::vector<std::string> &)));
    connect(this, SIGNAL(update_cal_list(const std::vector<std::string> &)), left_side,
            SLOT(update_cal_list(const std::vector<std::string> &)));

}

void SwagGUI::set_up_right() {
    // edit right side
    console_widget = new QPlainTextEdit;
    scan_controls = new ScanControls;
    calibrate_controls = new CalibrateControls;
    process_controls = new ProcessControls;
    options_combo_box = new QComboBox;

    options_combo_box->addItem("scan");
    options_combo_box->addItem("calibrate");
    options_combo_box->addItem("process");

    right_side->addWidget(options_combo_box);
    right_side->addWidget(scan_controls);
    right_side->addWidget(calibrate_controls);
    right_side->addWidget(process_controls);

    connect(options_combo_box, SIGNAL(currentIndexChanged(int)), this, SLOT(handle_combo_index_changed(int)));
    connect(scan_controls, SIGNAL(scan_button_pressed(const FormsPayload &)), this,
            SLOT(handle_scan_button_pressed(const FormsPayload &)));
    connect(calibrate_controls, SIGNAL(calibrate_button_pressed(const FormsPayload &)), this,
            SLOT(handle_calibrate_button_pressed(const FormsPayload &)));
    connect(process_controls, SIGNAL(process_button_pressed(const FormsPayload &)), this,
            SLOT(handle_process_button_pressed(const FormsPayload &)));


    calibrate_controls->setVisible(false);
    process_controls->setVisible(false);
//    right_side->addStretch();
    right_side->addWidget(console_widget);
}

SwagGUI::~SwagGUI() = default;


