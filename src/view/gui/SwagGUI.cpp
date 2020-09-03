#include "SwagGUI.h"
#include <QPushButton>
#include "SideBar.h"
#include <QHBoxLayout>
#include <QDesktopWidget>
#include <QComboBox>
#include <QPlainTextEdit>
#include <QSpacerItem>
#include <iostream>
#include "ScanControls.h"
#include "CalibrateControls.h"
#include "ProcessControls.h"

SwagGUI::SwagGUI(QWidget *parent)
        : QMainWindow(parent) {
    set_up_main();
    set_up_left();
    set_up_right();
}

std::string SwagGUI::get_name() {
    return name;
}

int SwagGUI::get_deg() {
    return deg;
}

int SwagGUI::get_rot() {
    return deg;
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

void SwagGUI::handle_scan_button_pressed(const std::vector<std::string> &vars) {
    name = vars[0];
    deg = stoi(vars[1]);
    rot = stoi(vars[2]);

    std::cout << name << std::endl;
    std::cout << deg << std::endl;
    std::cout << rot << std::endl;
    // call observer to run controller method
}

void SwagGUI::handle_calibrate_button_pressed(const std::vector<std::string> &vars) {
    name = vars[0];
    deg = stoi(vars[1]);
    rot = stoi(vars[2]);
    std::cout << name << std::endl;
    std::cout << deg << std::endl;
    std::cout << rot << std::endl;
    // call observer to run controller method
}

void SwagGUI::handle_process_button_pressed(const std::vector<std::string> &vars) {
    name = vars[0];
    std::cout << name << std::endl;
    // call observer to run controller method
}

void SwagGUI::handle_name_text_edited(const QString &text) {
    std::string utf8_text = text.toUtf8().constData();
    name = utf8_text;
    std::cout << utf8_text << std::endl;
}

void SwagGUI::handle_deg_text_edited(const QString &text) {
    std::string utf8_text = text.toUtf8().constData();
    deg = stoi(utf8_text);
    std::cout << utf8_text << std::endl;
}

void SwagGUI::handle_rot_text_edited(const QString &text) {
    std::string utf8_text = text.toUtf8().constData();
    rot = stoi(utf8_text);
    std::cout << utf8_text << std::endl;
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
    connect(scan_controls, SIGNAL(scan_button_pressed(const std::vector<std::string> &)), this,
            SLOT(handle_scan_button_pressed(const std::vector<std::string> &)));
    connect(calibrate_controls, SIGNAL(calibrate_button_pressed(const std::vector<std::string> &)), this,
            SLOT(handle_calibrate_button_pressed(const std::vector<std::string> &)));
    connect(process_controls, SIGNAL(process_button_pressed(const std::vector<std::string> &)), this,
            SLOT(handle_process_button_pressed(const std::vector<std::string> &)));


    calibrate_controls->setVisible(false);
    process_controls->setVisible(false);
//    right_side->addStretch();
    right_side->addWidget(console_widget);
}



