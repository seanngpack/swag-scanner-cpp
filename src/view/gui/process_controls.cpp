#include "process_controls.h"
#include <QLineEdit>
#include <QFormLayout>
#include <QPushButton>

ProcessControls::ProcessControls(QWidget *parent)
        : QWidget(parent) {
    v_layout = new QVBoxLayout;
    this->setLayout(v_layout);

    name_edit = new QLineEdit;
    form_layout = new QFormLayout;
    form_layout->addRow("Name:", name_edit);

    connect(name_edit, QOverload<const QString &>::of(&QLineEdit::textEdited), this,
            &ProcessControls::name_text_edited);

    calibrate_button = new QPushButton("process");

    v_layout->addLayout(form_layout);
    v_layout->addWidget(calibrate_button);

}