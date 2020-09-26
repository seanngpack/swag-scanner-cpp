//
// Created by Sean ng pack on 9/22/20.
//

#ifndef SWAG_SCANNER_SWAGGUI_H
#define SWAG_SCANNER_SWAGGUI_H

#include "MoveMethod.h"
#include <QMainWindow>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkRenderWindow.h>

class QThreadPool;

namespace controller {
    class ControllerManager;

    class IControllerGUI;
}

namespace Ui {
    class SwagGUI;
}

namespace pcl
{
    class PointXYZ;
    template<class pointT> class PointCloud;
}
template<class T>
using foo = boost::shared_ptr<pcl::PointCloud<T>>;

class SwagGUI : public QMainWindow {
Q_OBJECT

public:
    explicit SwagGUI(QWidget *parent = 0, controller::ControllerManager *manager = nullptr);

    ~SwagGUI();

    /**
     * Show the cloud in the vtk viewer. Does not support modifications yet.
     */
    void display_cloud(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud);

public Q_SLOTS:

    // calibration slots
    void on_calNameEdit_textChanged(const QString &text);

    void on_calDropdownBasic_angleSliderValueChanged(int value);

    void on_calDropdownBasic_rotationSliderValueChanged(int value);

    void on_runCalButton_clicked();

    // scan slots

    void on_scanNameEdit_textChanged(const QString &text);

    void on_scanDropdownBasic_angleSliderValueChanged(int value);

    void on_scanDropdownBasic_rotationSliderValueChanged(int value);

    void on_runScanButton_clicked();

    // process slots

    void on_processNameEdit_textChanged(const QString &text);

    void on_runProcessButton_clicked();

    // table slots

    /**
     * Edited does not call signal when set programatically. Need this behavior to link
     * the moveTo and moveBy signals to each other.
     */
    void on_moveByEdit_textEdited(const QString &text);

    void on_moveToEdit_textEdited(const QString &text);

    void on_moveButton_clicked();

    void on_setHomeButton_clicked();


protected:
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

private:
    QThreadPool *thread_pool;

    Ui::SwagGUI *ui;
    controller::ControllerManager *manager;

    // scan_tab component state variables
    QString scan_name;
    int scan_angle;
    int scan_rotations;

    // cal_tab component state variables
    QString cal_name;
    int cal_angle;
    int cal_rotations;

    // process_tab component state varibles
    QString process_name;

    // table_tab copmopnent state variables
    MoveMethod move_method;
    int move_deg;

    //
};


#endif //SWAG_SCANNER_SWAGGUI_H
