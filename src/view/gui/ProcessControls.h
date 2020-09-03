#ifndef SWAG_SCANNER_PROCESSCONTROLS_H
#define SWAG_SCANNER_PROCESSCONTROLS_H


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

    void process_button_pressed(const std::vector<std::string> &vars);

private slots:
    void send_process_button_pressed();

private:
    QVBoxLayout *v_layout;
    QFormLayout *form_layout;

    QLineEdit *name_edit;

    QPushButton *process_button;
};


#endif //SWAG_SCANNER_PROCESSCONTROLS_H
