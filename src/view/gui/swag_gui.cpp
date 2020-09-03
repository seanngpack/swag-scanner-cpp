#include "swag_gui.h"
#include <QPushButton>
#include "sidebar.h"
#include <QHBoxLayout>
#include <QDesktopWidget>
#include <QComboBox>
#include <QPlainTextEdit>
#include <QSpacerItem>
#include <iostream>
#include "scan_controls.h"
#include "calibrate_controls.h"
#include "process_controls.h"

swag_gui::swag_gui(QWidget *parent)
        : QMainWindow(parent) {
    set_up_main();
    set_up_left();
    set_up_right();
}

void swag_gui::handle_combo_index_changed(int index) {
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

void swag_gui::handle_name_text_edited(const QString &text) {
    std::string utf8_text = text.toUtf8().constData();
    std::cout << utf8_text << std::endl;
}

void swag_gui::handle_deg_text_edited(const QString &text) {
    std::string utf8_text = text.toUtf8().constData();
    std::cout << utf8_text << std::endl;
}

void swag_gui::handle_rot_text_edited(const QString &text) {
    std::string utf8_text = text.toUtf8().constData();
    std::cout << utf8_text << std::endl;
}

void swag_gui::set_up_main() {
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

void swag_gui::set_up_left() {
    connect(this, SIGNAL(update_scan_list(const std::vector<std::string> &)), left_side,
            SLOT(update_scan_list(const std::vector<std::string> &)));
    connect(this, SIGNAL(update_cal_list(const std::vector<std::string> &)), left_side,
            SLOT(update_cal_list(const std::vector<std::string> &)));

}

void swag_gui::set_up_right() {
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

    connect(scan_controls, SIGNAL(name_text_edited(const QString &)), this,
            SLOT(handle_name_text_edited(const QString &)));
    connect(scan_controls, SIGNAL(deg_text_edited(const QString &)), this,
            SLOT(handle_deg_text_edited(const QString &)));
    connect(scan_controls, SIGNAL(rot_text_edited(const QString &)), this,
            SLOT(handle_rot_text_edited(const QString &)));

    connect(calibrate_controls, SIGNAL(name_text_edited(const QString &)), this,
            SLOT(handle_name_text_edited(const QString &)));
    connect(calibrate_controls, SIGNAL(deg_text_edited(const QString &)), this,
            SLOT(handle_deg_text_edited(const QString &)));
    connect(calibrate_controls, SIGNAL(rot_text_edited(const QString &)), this,
            SLOT(handle_rot_text_edited(const QString &)));

    connect(process_controls, SIGNAL(name_text_edited(const QString &)), this,
            SLOT(handle_name_text_edited(const QString &)));


    calibrate_controls->setVisible(false);
    process_controls->setVisible(false);
//    right_side->addStretch();
    right_side->addWidget(console_widget);
}
