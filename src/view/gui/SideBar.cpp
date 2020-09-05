#include "SideBar.h"
#include <QComboBox>
#include <QVBoxLayout>
#include <QPlainTextEdit>

SideBar::SideBar(QWidget *parent)
        : QWidget(parent) {

    layout = new QVBoxLayout;
    scan_cal_combo = new QComboBox;
    scan_cal_combo->addItem("Previous scans");
    scan_cal_combo->addItem("Previous calibrations");

    connect(scan_cal_combo, SIGNAL(currentIndexChanged(int)), this, SLOT(handle_scan_cal_combo_changed(int)));
    connect(scan_cal_combo, SIGNAL(currentIndexChanged(int)), this, SLOT(send_scan_cal_combo_changed(int)));

    scan_list = new QPlainTextEdit;
    cal_list = new QPlainTextEdit;

    layout->addWidget(scan_cal_combo);
    layout->addWidget(scan_list);
    layout->addWidget(cal_list);
//    layout->addStretch();

    cal_list->setVisible(false);

    this->setLayout(layout);
//    this->show();
}

void SideBar::send_scan_cal_combo_changed(int index) {
    emit scan_cal_combo_changed(index);
}

void SideBar::handle_scan_cal_combo_changed(int index) {
    if (index == 1) {
        scan_list->setVisible(false);
        cal_list->setVisible(true);
        cal_list->clear();

    } else {
        scan_list->setVisible(true);
        cal_list->setVisible(false);
        scan_list->clear();
    }
}

void SideBar::update_scan_list(const std::vector<std::string> &scans) {
    for (const auto &scan: scans) {
        scan_list->appendPlainText(QString::fromStdString(scan));
    }
}

void SideBar::update_cal_list(const std::vector<std::string> &scans) {
    for (const auto &scan: scans) {
        cal_list->appendPlainText(QString::fromStdString(scan));
    }
}
