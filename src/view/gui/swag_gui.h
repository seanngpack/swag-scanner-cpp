#ifndef SWAG_SCANNER_SWAG_GUI_H
#define SWAG_SCANNER_SWAG_GUI_H


#include <QMainWindow>

class QPlainTextEdit;

class QVBoxLayout;

class QHBoxLayout;

class QComboBox;

class swag_gui : public QMainWindow {
Q_OBJECT
public:
    explicit swag_gui(QWidget *parent = 0);

signals:

    void update_scan_list(const std::vector<std::string> &input);

    void update_cal_list(const std::vector<std::string> &input);

private slots:

    void handle_combo_index_changed(int index);

    void handle_name_text_edited(const QString &text);

    void handle_deg_text_edited(const QString &text);

    void handle_rot_text_edited(const QString &text);

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


    void set_up_main();

    void set_up_left();

    void set_up_right();


};

#endif //SWAG_SCANNER_SWAG_GUI_H
