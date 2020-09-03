#ifndef SWAG_SCANNER_SWAGGUI_H
#define SWAG_SCANNER_SWAGGUI_H


#include <QMainWindow>
#include "IControllerGUI.h" // can probably remove this one
#include "CalibrationControllerGUI.h" // i think it's important to include this one

class QPlainTextEdit;

class QVBoxLayout;

class QHBoxLayout;

class QComboBox;

class FormsPayload;

class SwagGUI : public QMainWindow {
Q_OBJECT
public:
    explicit SwagGUI(std::shared_ptr<controller::IControllerGUI> controller, QWidget *parent = 0);

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

    void update_scan_list(const std::vector<std::string> &input);

    void update_cal_list(const std::vector<std::string> &input);

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


private:
    std::shared_ptr<controller::IControllerGUI> controller;

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
