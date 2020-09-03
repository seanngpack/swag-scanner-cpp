#include "scan_controls.h"
#include <QFormLayout>
#include <QLineEdit>
#include <QPushButton>

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

    connect(name_edit, QOverload<const QString &>::of(&QLineEdit::textEdited), this, &ScanControls::name_text_edited);
    connect(deg_edit, QOverload<const QString &>::of(&QLineEdit::textEdited), this, &ScanControls::deg_text_edited);
    connect(rot_edit, QOverload<const QString &>::of(&QLineEdit::textEdited), this, &ScanControls::rot_text_edited);

    scan_button = new QPushButton("scan");

    v_layout->addLayout(form_layout);
    v_layout->addWidget(scan_button);

}