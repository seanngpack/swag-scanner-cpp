#ifndef SWAG_SCANNER_TEMPGUI_H
#define SWAG_SCANNER_TEMPGUI_H


#include <QMainWindow>


class QPlainTextEdit;

class QVBoxLayout;

class QHBoxLayout;

class QComboBox;

class FormsPayload;

class MoveFormsPayload;

class QThreadPool;

namespace controller {
    class ControllerManager;

    class IControllerGUI;
}

class TEMPGUI : public QMainWindow {
Q_OBJECT
public:
    explicit TEMPGUI(controller::ControllerManager *factory, QWidget *parent = 0);

    ~TEMPGUI();

//    void set_controller(controller::IControllerGUI *c);


signals:

    /**
     * Send signal to update the scans in the left bar scan list.
     *
     * @param scans vector of scan names.
     */
    void update_scan_list(const std::vector<std::string> &scans);

    /**
     * Send signal to update the calibrations in the left bar scan list.
     *
     * @param scans vector of calibration names.
     */
    void update_cal_list(const std::vector<std::string> &scans);


private slots:

    /**
     * Handler what happens when the "scan" button is pressed.
     *
     * @param vars vector containing scan, rot, and deg text fields.
     */
    void handle_scan_button_pressed(const FormsPayload &vars);

    /**
     * When the "calibrate" button is pressed, will retrieve a calibration controller from the manager,
     * update the controller with form parameters, and finally call the run() method.
     *
     * @param vars payload variables.
     */
    void handle_calibrate_button_pressed(const FormsPayload &vars);

    /**
     * Handle what happens when "process" button is pressed.
     *
     * @param vars
     */
    void handle_process_button_pressed(const FormsPayload &vars);


    /**
     * When the "move" button is pressed, will retrieve a move controller from the manager,
     * update the controller with form parameters, and finall call the run() method.
     * If both the move_to and move_by forms are filled, then it will use the move_to value.
     *
     * @param vars
     */
    void handle_move_button_pressed(const MoveFormsPayload &vars);

    /**
     * Set the home position to current position when the "set home" button is pressed.
     */
    void handle_set_home_button_pressed();

    /**
     * Handle when combobox for selecting "scan", "calibrate", or "process" changes.
     * @param index
     */
    void handle_combo_index_changed(int index);

    /**
     * Handler when the previous scans section for "scan" or "calibrate" changes.
     * Will emit update_scan_list or update_cal_list signal
     * @param index
     */
    void handle_scan_cal_combo_changed(int index);

    /**
     * Append info to the console. This method is called from a threaded worker.
     *
     * @param info the information you want to append.
     */
    void update_console(const std::string &info);


private:
    controller::ControllerManager *factory;
//    controller::IControllerGUI *controller;

    QThreadPool *thread_pool;
    QWidget *main;
    QHBoxLayout *main_layout;
    QWidget *left_side;
    QVBoxLayout *right_side;
    QComboBox *options_combo_box;
    QPlainTextEdit *console_widget;
    QWidget *scan_controls;
    QWidget *calibrate_controls;
    QWidget *process_controls;
    QWidget *move_controls;

    void set_up_main();

    void set_up_left();

    void set_up_right();


};

#endif //SWAG_SCANNER_TEMPGUI_H
