#include "ProcessControls.h"
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

    process_button = new QPushButton("process");

    connect(process_button, SIGNAL(pressed()), this, SLOT(send_process_button_pressed()));

    v_layout->addLayout(form_layout);
    v_layout->addWidget(process_button);

}

void ProcessControls::send_process_button_pressed() {
    emit process_button_pressed(std::vector<std::string>{
            name_edit->text().toUtf8().constData()
    });
}