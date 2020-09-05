#ifndef SWAG_SCANNER_SIDEBAR_H
#define SWAG_SCANNER_SIDEBAR_H

#include <QWidget>

class QVBoxLayout;

class QPushButton;

class QComboBox;

class QPlainTextEdit;

namespace controller {
    class IControllerGUI;
}

class SideBar : public QWidget {
Q_OBJECT
public:
    explicit SideBar(QWidget *parent = 0);

public slots:

    void handle_scan_cal_combo_changed(int index);

private slots:

    /**
     * Emit scan_cal_combo_changed signal to parent class.
     * @param index current index of combo box.
     */
    void send_scan_cal_combo_changed(int index);

    /**
     * Update the scan list.
     * @param scans
     */
    void update_scan_list(const std::vector<std::string> &scans);

    /**
     * Update the cal list.
     * @param scans
     */
    void update_cal_list(const std::vector<std::string> &scans);

signals:


    void scan_cal_combo_changed(int index);


private:
    QVBoxLayout *layout;
    QComboBox *scan_cal_combo;
    QPlainTextEdit *cal_list;
    QPlainTextEdit *scan_list;


};

#endif //SWAG_SCANNER_SIDEBAR_H
