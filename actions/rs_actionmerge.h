#ifndef RS_ACTIONMERGE_H
#define RS_ACTIONMERGE_H

#include <QDialog>

namespace Ui {
class rs_actionmerge;
}

class rs_actionmerge : public QDialog
{
    Q_OBJECT

public:
    explicit rs_actionmerge(QWidget *parent = nullptr);
    ~rs_actionmerge();
private:
    Ui::rs_actionmerge *ui;
    //声明信号函数
private slots:
    void okSlot();
    void cancelSlot();
signals:
    void myMergeSignal(double data);
    void closeWindowSignal(bool);

protected:
    virtual void closeEvent(QCloseEvent *event) override;

};

#endif // RS_ACTIONMERGE_H
