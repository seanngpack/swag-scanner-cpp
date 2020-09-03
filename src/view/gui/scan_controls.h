#ifndef SWAG_SCANNER_SCAN_CONTROLS_H
#define SWAG_SCANNER_SCAN_CONTROLS_H


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

signals:
    void name_text_edited(QString text);
    void deg_text_edited(QString text);
    void rot_text_edited(QString text);

private:
    QVBoxLayout *v_layout;
    QHBoxLayout *h_layout;
    QFormLayout *form_layout;

    QLineEdit *name_edit;
    QLineEdit *rot_edit;
    QLineEdit *deg_edit;

    QPushButton *scan_button;

};

#endif //SWAG_SCANNER_SCAN_CONTROLS_H
