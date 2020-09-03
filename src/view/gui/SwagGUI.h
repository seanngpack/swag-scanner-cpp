#ifndef SWAG_SCANNER_SWAGGUI_H
#define SWAG_SCANNER_SWAGGUI_H


#include <QMainWindow>

class QPlainTextEdit;

class QVBoxLayout;

class QHBoxLayout;

class QComboBox;

class FormsPayload;

class SwagGUI : public QMainWindow {
Q_OBJECT
public:
    explicit SwagGUI(QWidget *parent = 0);

    /**
     * Get the name text in the current form.
     * @return name.
     */
    std::string get_name();

    /**
     * Get the degree text in the current form.
     * @return degree.
     */
    int get_deg();

    /**
     * Get the number of rotations in the current form.
     * @return num of rotations.
     */
    int get_rot();



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
