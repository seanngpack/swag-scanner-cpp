#include "ScanControls.h"
#include <QFormLayout>
#include <QLineEdit>
#include <QPushButton>
#include <iostream>

ScanControls::ScanControls(QWidget *parent)
        : QWidget(parent) {
    v_layout = new QVBoxLayout;
    this->setLayout(v_layout);

    name_edit = new QLineEdit;
    deg_edit = new QLineEdit;
    rot_edit = new QLineEdit;

    form_layout = new QFormLayout;

    form_layout->addRow("Name:", name_edit);
    form_layout->addRow("Degrees:", deg_edit);
    form_layout->addRow("# Rotations: ", rot_edit);

    scan_button = new QPushButton("scan");

    connect(scan_button, SIGNAL(pressed()), this, SLOT(send_scan_button_pressed()));

    v_layout->addLayout(form_layout);
    v_layout->addWidget(scan_button);

}

std::string ScanControls::get_name() {
    return name_edit->text().toUtf8().constData();
}

void ScanControls::send_scan_button_pressed() {
    emit scan_button_pressed(std::vector<std::string>{
        name_edit->text().toUtf8().constData(),
        deg_edit->text().toUtf8().constData(),
        rot_edit->text().toUtf8().constData()
    });

}
