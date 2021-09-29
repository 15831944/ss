#ifndef SX_INTERACTIVEARRAY_H
#define SX_INTERACTIVEARRAY_H

#include <QDialog>
#include "QMessageBox"
#include "QDoubleValidator"

namespace Ui {
class sx_interactiveArray;
}

class sx_interactiveArray : public QDialog
{
    Q_OBJECT

public:
    explicit sx_interactiveArray(QWidget *parent = nullptr);
    ~sx_interactiveArray();

private:
    Ui::sx_interactiveArray *ui;

private slots:
    void slotClickOk();
    void slotCloseWindow();

signals:
    void sendDataSignal(bool,bool,double,double);
    void closeWindowSignal(bool isClose = false);

protected:
    virtual void closeEvent(QCloseEvent * event);
};

#endif // SX_INTERACTIVEARRAY_H
