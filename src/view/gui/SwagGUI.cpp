#include "ControllerManager.h"
#include "SwagGUI.h"
#include "ui_swagscannerview.h"
#include "MoveFormsPayload.h"
#include "FormsPayload.h"
#include "IControllerGUI.h"
#include "MoveControllerGUI.h"
#include "IFileHandler.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <QThread>

SwagGUI::SwagGUI(QWidget *parent, controller::ControllerManager *manager) :
        QMainWindow(parent),
        manager(manager),
        ui(new Ui::SwagGUI) {
    ui->setupUi(this);
    this->setWindowTitle("Swag Scanner");

    thread_pool = QThreadPool::globalInstance();

    // set up viewer
    viewer = std::make_shared<pcl::visualization::PCLVisualizer>("viewer", false);
    ui->cloud_viewer->SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(ui->cloud_viewer->GetInteractor(), ui->cloud_viewer->GetRenderWindow());
    ui->cloud_viewer->update();

    // set up basic calibration info
    cal_angle = ui->calDropdownBasic->get_angle_slider_value() * 3;
    cal_rotations = ui->calDropdownBasic->get_rotation_slider_value();

    // set up basic scan info
    scan_angle = ui->scanDropdownBasic->get_angle_slider_value() * 3;
    scan_rotations = ui->scanDropdownBasic->get_rotation_slider_value();

    // set up left widget stack current page to scanning
    ui->leftStack->setCurrentIndex(2);
}

SwagGUI::~SwagGUI() {
    delete ui;
}

// --------------------------------------------------------------------------------
//                          Public methods
// --------------------------------------------------------------------------------

void SwagGUI::display_cloud(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud) {
    viewer->removeAllPointClouds();
    viewer->addPointCloud(cloud, "cloud");
    viewer->resetCamera();
    ui->cloud_viewer->update();
}

// --------------------------------------------------------------------------------
//                          Calibration slots
// --------------------------------------------------------------------------------

void SwagGUI::on_calNameEdit_textChanged(const QString &text) {
    cal_name = text;
}

void SwagGUI::on_calDropdownBasic_angleSliderValueChanged(int value) {
    cal_angle = value * 3;
}

void SwagGUI::on_calDropdownBasic_rotationSliderValueChanged(int value) {
    cal_rotations = value;
}

void SwagGUI::on_runCalButton_clicked() {
    controller::IControllerGUI *c = manager->get_gui_controller("calibrate").get();
    FormsPayload vars(cal_name, cal_angle, cal_rotations);
    std::cout << vars.name << std::endl;
    std::cout << vars.angle << std::endl;
    std::cout << vars.rotations << std::endl;
    c->update(vars);
    c->setAutoDelete(false);
    // dont need this for now as the GUI may not have its own console
//    connect(c, SIGNAL(update_console(const std::string &)), this, SLOT(update_console(const std::string &)),
//            Qt::UniqueConnection);
    thread_pool->start(c);
}

// --------------------------------------------------------------------------------
//                          Scan slots
// --------------------------------------------------------------------------------

void SwagGUI::on_scanNameEdit_textChanged(const QString &text) {
    scan_name = text;
}

void SwagGUI::on_scanDropdownBasic_angleSliderValueChanged(int value) {
    scan_angle = value * 3;
}

void SwagGUI::on_scanDropdownBasic_rotationSliderValueChanged(int value) {
    scan_rotations = value;
}

void SwagGUI::on_runScanButton_clicked() {
    controller::IControllerGUI *c = manager->get_gui_controller("scan").get();
    FormsPayload vars(scan_name, scan_angle, scan_rotations);
    c->update(vars);
    c->setAutoDelete(false);
//    connect(c, SIGNAL(update_console(const std::string &)), this, SLOT(update_console(const std::string &)),
//            Qt::UniqueConnection);
    thread_pool->start(c);
}

// --------------------------------------------------------------------------------
//                          Processing slots
// --------------------------------------------------------------------------------
void SwagGUI::on_processNameEdit_textChanged(const QString &text) {
    process_name = text;
}


void SwagGUI::on_runProcessButton_clicked() {
    controller::IControllerGUI *c = manager->get_gui_controller("process").get();
    FormsPayload vars(process_name, 0, 0);
    c->update(vars);
    c->setAutoDelete(false);
//    connect(c, SIGNAL(update_console(const std::string &)), this, SLOT(update_console(const std::string &)),
//            Qt::UniqueConnection);
    thread_pool->start(c);
}

void SwagGUI::on_moveByEdit_textEdited(const QString &text) {
    ui->moveToEdit->setText("nan");
    move_deg = text.toInt();
    move_method = MoveMethod::BY;
}

void SwagGUI::on_moveToEdit_textEdited(const QString &text) {
    ui->moveByEdit->setText("nan");
    move_deg = text.toInt();
    move_method = MoveMethod::TO;
    // set move_by to 0
}

void SwagGUI::on_moveButton_clicked() {
    controller::IControllerGUI *c = manager->get_gui_controller("move").get();
    MoveFormsPayload vars(move_method, move_deg);
    c->update(vars);
    std::cout << move_deg << std::endl;
    c->setAutoDelete(false);
//    connect(c, SIGNAL(update_console(const std::string &)), this, SLOT(update_console(const std::string &)),
//            Qt::UniqueConnection);
    thread_pool->start(c);
    thread_pool->waitForDone();
}

void SwagGUI::on_setHomeButton_clicked() {
    controller::IControllerGUI *c = manager->get_gui_controller("move").get();
    // intentially casting because the alternative would be to use a HomeController class
    // where the run() method sets home. I think this is cleaner.
    dynamic_cast<controller::MoveControllerGUI *>(c)->set_home();
}

// --------------------------------------------------------------------------------
//                          edit slots
// --------------------------------------------------------------------------------

void SwagGUI::on_openProjectButton_clicked() {
    QString dir = QFileDialog::getExistingDirectory(this, tr("Open Directory"),
                                                    QString::fromStdString(file::IFileHandler::swag_scanner_path.string() + "/scans"),
                                                    QFileDialog::ShowDirsOnly
                                                    | QFileDialog::DontResolveSymlinks);
    std::cout << dir.toUtf8().constData() << std::endl;

    controller::IControllerGUI *c = manager->get_gui_controller("edit").get();
    std::cout << c << std::endl;

    // TODO: fix this horrible way to move data lol
    FormsPayload vars(dir.toUtf8().constData(), 0,0);
    c->update(vars);
    c->setAutoDelete(false);
    thread_pool->start(c);
}

void SwagGUI::on_saveProjectButton_clicked() {

}

// --------------------------------------------------------------------------------
//                          Menu slots
// --------------------------------------------------------------------------------

void SwagGUI::on_setupMenuButton_clicked() {
    ui->leftStack->setCurrentIndex(0);
}

void SwagGUI::on_calibrateMenuButton_clicked() {
    ui->leftStack->setCurrentIndex(1);
}

void SwagGUI::on_scanMenuButton_clicked() {
    ui->leftStack->setCurrentIndex(2);
}

void SwagGUI::on_processMenuButton_clicked() {
    ui->leftStack->setCurrentIndex(3);
}

void SwagGUI::on_editMenuButton_clicked() {
    ui->leftStack->setCurrentIndex(4);
}

