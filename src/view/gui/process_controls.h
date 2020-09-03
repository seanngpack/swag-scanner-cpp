#ifndef SWAG_SCANNER_PROCESS_CONTROLS_H
#define SWAG_SCANNER_PROCESS_CONTROLS_H


#include <QWidget>

class QVBoxLayout;

class QFormLayout;

class QLineEdit;

class QPushButton;

class ProcessControls : public QWidget {

Q_OBJECT
public:
    explicit ProcessControls(QWidget *parent = 0);

signals:
    void name_text_edited(QString text);

private:
    QVBoxLayout *v_layout;
    QFormLayout *form_layout;

    QLineEdit *name_edit;

    QPushButton *calibrate_button;

};


#endif //SWAG_SCANNER_PROCESS_CONTROLS_H
