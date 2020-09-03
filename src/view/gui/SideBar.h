#ifndef SWAG_SCANNER_SIDEBAR_H
#define SWAG_SCANNER_SIDEBAR_H

#include <QWidget>

class QVBoxLayout;

class QPushButton;

class QComboBox;

class QPlainTextEdit;

class SideBar : public QWidget {
Q_OBJECT
public:
    explicit SideBar(QWidget *parent = 0);

public slots:

    void update_scan_list(const std::vector<std::string> &input);
    void update_cal_list(const std::vector<std::string> &input);
    void handle_scan_cal_combo_changed(int index);

signals:

    void scan_cal_combo_changed(int index);


private:
    QVBoxLayout *layout;
    QComboBox *scan_cal_combo;
    QPlainTextEdit *cal_list;
    QPlainTextEdit *scan_list;



};
#endif //SWAG_SCANNER_SIDEBAR_H
