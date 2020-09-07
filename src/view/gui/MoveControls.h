#ifndef SWAG_SCANNER_MOVECONTROLS_H
#define SWAG_SCANNER_MOVECONTROLS_H

#include <QWidget>

class QVBoxLayout;

class QFormLayout;

class QLineEdit;

class QPushButton;

class MoveFormsPayload;

class MoveControls : public QWidget {
Q_OBJECT
public:
    explicit MoveControls(QWidget *parent = 0);

signals:

    void move_button_pressed(const MoveFormsPayload &vars);

private slots:

    void send_move_button_pressed();

private:
    QVBoxLayout *v_layout;
    QFormLayout *form_layout;
    QLineEdit *to_edit;
    QLineEdit *by_edit;
    QPushButton *move_button;

};

#endif //SWAG_SCANNER_MOVECONTROLS_H
