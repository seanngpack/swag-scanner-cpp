#include "ControllerManager.h"
#include "SwagGUI.h"
#include "ui_swagscannerview.h"

SwagGUI::SwagGUI(QWidget *parent, controller::ControllerManager *manager) :
        QMainWindow(parent),
        manager(manager),
        ui(new Ui::SwagGUI) {
    ui->setupUi(this);
    this->setWindowTitle("Swag Scanner");

    // set up viewer
    viewer = std::make_shared<pcl::visualization::PCLVisualizer>("viewer", false);
    ui->cloud_viewer->SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(ui->cloud_viewer->GetInteractor(), ui->cloud_viewer->GetRenderWindow());
    ui->cloud_viewer->update();

}

SwagGUI::~SwagGUI() {
    delete ui;
}