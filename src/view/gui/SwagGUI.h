//
// Created by Sean ng pack on 9/22/20.
//

#ifndef SWAG_SCANNER_SWAGGUI_H
#define SWAG_SCANNER_SWAGGUI_H

#include <QMainWindow>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkRenderWindow.h>

namespace controller {
    class ControllerManager;

    class IControllerGUI;
}

namespace Ui {
    class SwagGUI;
}

class SwagGUI : public QMainWindow {
Q_OBJECT

public:
    explicit SwagGUI(QWidget *parent = 0, controller::ControllerManager *manager = nullptr);

    ~SwagGUI();

public Q_SLOTS:
//    void on


protected:
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

private:
    Ui::SwagGUI *ui;
    controller::ControllerManager *manager;
    //

    //
};


#endif //SWAG_SCANNER_SWAGGUI_H
