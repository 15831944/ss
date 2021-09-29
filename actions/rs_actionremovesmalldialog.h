#ifndef RS_ACTIONREMOVESMALLDIALOG_H
#define RS_ACTIONREMOVESMALLDIALOG_H

#include <QDialog>

namespace Ui {
class rs_actionremovesmalldialog;
}

class rs_actionremovesmalldialog : public QDialog
{
    Q_OBJECT

public:
    explicit rs_actionremovesmalldialog(QWidget *parent = nullptr);
    ~rs_actionremovesmalldialog();


private:
    Ui::rs_actionremovesmalldialog *ui;
signals:
    void myRemoveSmallSignal(double data, int flag);
    void closeWindowSignal(bool);
private slots:
    void lineeditSlot(QString text);
    void accept1();
    void reject1();


protected:
    virtual void closeEvent(QCloseEvent *event) override;
};

#endif // RS_ACTIONREMOVESMALLDIALOG_H
