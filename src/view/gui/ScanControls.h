#ifndef SWAG_SCANNER_SCANCONTROLS_H
#define SWAG_SCANNER_SCANCONTROLS_H


#include <QWidget>

class QVBoxLayout;

class QHBoxLayout;

class QFormLayout;

class QLineEdit;

class QPushButton;

class ScanControls : public QWidget {

Q_OBJECT
public:
    explicit ScanControls(QWidget *parent = 0);

    std::string get_name();

signals:

    void scan_button_pressed(const std::vector<std::string> &vars);

private slots:
    void send_scan_button_pressed();


private:
    QVBoxLayout *v_layout;
    QFormLayout *form_layout;

    QLineEdit *name_edit;
    QLineEdit *rot_edit;
    QLineEdit *deg_edit;

    QPushButton *scan_button;


};

#endif //SWAG_SCANNER_SCANCONTROLS_H
