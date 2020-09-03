#ifndef SWAG_SCANNER_CALIBRATE_CONTROLS_H
#define SWAG_SCANNER_CALIBRATE_CONTROLS_H


#include <QWidget>

class QVBoxLayout;

class QFormLayout;

class QLineEdit;

class QPushButton;

class CalibrateControls : public QWidget {

Q_OBJECT
public:
    explicit CalibrateControls(QWidget *parent = 0);

signals:

    void name_text_edited(QString text);

    void deg_text_edited(QString text);

    void rot_text_edited(QString text);

private:
    QVBoxLayout *v_layout;
    QFormLayout *form_layout;

    QLineEdit *name_edit;
    QLineEdit *rot_edit;
    QLineEdit *deg_edit;

    QPushButton *calibrate_button;

};


#endif //SWAG_SCANNER_CALIBRATE_CONTROLS_H
