#ifndef SWAG_SCANNER_CALIBRATECONTROLS_H
#define SWAG_SCANNER_CALIBRATECONTROLS_H


#include <QWidget>

class QVBoxLayout;

class QFormLayout;

class QLineEdit;

class QPushButton;

class FormsPayload;

class CalibrateControls : public QWidget {

Q_OBJECT
public:
    explicit CalibrateControls(QWidget *parent = 0);

signals:

    void calibrate_button_pressed(const FormsPayload &vars);

private slots:

    void send_calibrate_button_pressed();

private:
    QVBoxLayout *v_layout;
    QFormLayout *form_layout;

    QLineEdit *name_edit;
    QLineEdit *rot_edit;
    QLineEdit *deg_edit;

    QPushButton *calibrate_button;

};


#endif //SWAG_SCANNER_CALIBRATECONTROLS_H
