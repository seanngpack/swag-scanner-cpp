#ifndef SWAG_SCANNER_SWAGGUI_H
#define SWAG_SCANNER_SWAGGUI_H


#include <QMainWindow>


class QPlainTextEdit;

class QVBoxLayout;

class QHBoxLayout;

class QComboBox;

class FormsPayload;

namespace controller {
    class ControllerFactory;

    class IControllerGUI;
}

class SwagGUI : public QMainWindow {
Q_OBJECT
public:
    explicit SwagGUI(controller::ControllerFactory *factory, QWidget *parent = 0);

    ~SwagGUI();

    void set_controller(controller::IControllerGUI *c);

    /**
     * Get the name text in the current form.
     * @return name.
     */
    std::string update_name() const;

    /**
     * Get the degree text in the current form.
     * @return degree.
     */
    int update_deg() const;

    /**
     * Get the number of rotations in the current form.
     * @return num of rotations.
     */
    int update_rot() const;

    /**
     * Append info to the console.
     * @param info the information you want to append.
     */
    void update_console(const std::string &info);


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
     * @param vars vector containing scan, rot, and deg text fields.
     */
    void handle_scan_button_pressed(const FormsPayload &vars);

    void handle_calibrate_button_pressed(const FormsPayload &vars);

    void handle_process_button_pressed(const FormsPayload &vars);

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


private:
    controller::ControllerFactory *factory;
    controller::IControllerGUI *controller;

    QWidget *main;
    QHBoxLayout *main_layout;
    QWidget *left_side;
    QVBoxLayout *right_side;
    QComboBox *options_combo_box;
    QPlainTextEdit *console_widget;
    QWidget *scan_controls;
    QWidget *calibrate_controls;
    QWidget *process_controls;

    std::string name;
    int deg = 0;
    int rot = 0;


    void set_up_main();

    void set_up_left();

    void set_up_right();


};

#endif //SWAG_SCANNER_SWAGGUI_H
