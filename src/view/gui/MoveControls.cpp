#include "MoveControls.h"
#include "MoveFormsPayload.h"
#include <QLineEdit>
#include <QFormLayout>
#include <QPushButton>
#include <QLabel>

MoveControls::MoveControls(QWidget *parent)
        : QWidget(parent) {
    v_layout = new QVBoxLayout;
    this->setLayout(v_layout);

    to_edit = new QLineEdit;
    by_edit = new QLineEdit;
    form_layout = new QFormLayout;
    form_layout->addRow("move to position: ", to_edit);
    auto *or_label = new QLabel("OR");
    form_layout->addRow(or_label);
    form_layout->addRow("move by degrees: ", by_edit);

    move_button = new QPushButton("move");
    set_home_button = new QPushButton("set home");

    connect(move_button, SIGNAL(pressed()), this, SLOT(send_move_button_pressed()));
    connect(set_home_button, SIGNAL(pressed()), this, SLOT(send_set_home_button_pressed()));

    v_layout->addLayout(form_layout);
    v_layout->addWidget(move_button);
    v_layout->addWidget(set_home_button);

}

void MoveControls::send_move_button_pressed() {
    emit move_button_pressed(MoveFormsPayload(to_edit->text(), by_edit->text()));
}

void MoveControls::send_set_home_button_pressed() {
    emit set_home_button_pressed();
}