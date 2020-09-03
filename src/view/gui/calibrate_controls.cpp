#include "calibrate_controls.h"
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

    connect(name_edit, QOverload<const QString &>::of(&QLineEdit::textEdited), this, &CalibrateControls::name_text_edited);
    connect(deg_edit, QOverload<const QString &>::of(&QLineEdit::textEdited), this, &CalibrateControls::deg_text_edited);
    connect(rot_edit, QOverload<const QString &>::of(&QLineEdit::textEdited), this, &CalibrateControls::rot_text_edited);

    calibrate_button = new QPushButton("calibrate");

    v_layout->addLayout(form_layout);
    v_layout->addWidget(calibrate_button);

}