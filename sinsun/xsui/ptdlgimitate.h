#ifndef PTDLGIMITATE_H
#define PTDLGIMITATE_H

#include <QDialog>

#include "sinsun/xstest/ncfile.h"
#include "sinsun/xstest/ptshowncfile.h"

class ptAnimationThread;

namespace Ui {
class ptdlgimitate;
}

class ptdlgimitate : public QDialog
{
    Q_OBJECT

public:
    explicit ptdlgimitate(QWidget *parent = nullptr);
    ~ptdlgimitate();

    std::shared_ptr<ncFileData> m_ncfile;

public:
    void  pt_begin_thread();

protected:
     void closeEvent(QCloseEvent *event);

private slots:
    void on_pushButtonStart_clicked();

    void on_pushButtonTimeOut_clicked();

    void on_pushButtonSlowDown_clicked();

    void on_pushButtonAccelerate_clicked();

    void on_pushButtonWalkBorder_clicked();

    void on_pushButtonStop_clicked();

    void on_horizontalSliderScheduleContral_valueChanged(int value);

private:
    Ui::ptdlgimitate *ui;

    ptAnimationThread* m_animation;

    int m_speed_contral=0;

};

#endif // PTDLGIMITATE_H
