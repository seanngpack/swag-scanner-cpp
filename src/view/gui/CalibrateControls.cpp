#include "CalibrateControls.h"
#include <QFormLayout>
#include <QLineEdit>
#include <QPushButton>

CalibrateControls::CalibrateControls(QWidget *parent)
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


    calibrate_button = new QPushButton("calibrate");

    connect(calibrate_button, SIGNAL(pressed()), this, SLOT(send_calibrate_button_pressed()));
    v_layout->addLayout(form_layout);
    v_layout->addWidget(calibrate_button);

}

void CalibrateControls::send_calibrate_button_pressed() {
    emit calibrate_button_pressed(std::vector<std::string>{
            name_edit->text().toUtf8().constData(),
            deg_edit->text().toUtf8().constData(),
            rot_edit->text().toUtf8().constData()
    });

}
