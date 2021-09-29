#ifndef PTDLGCOLDPOINTS_H
#define PTDLGCOLDPOINTS_H

#include <QDialog>
#include "sinsun/xstest/ncfile.h"

namespace Ui {
class ptdlgcoldpoints;
}

class ptdlgcoldpoints : public QDialog
{
    Q_OBJECT

public:
    explicit ptdlgcoldpoints(QWidget *parent = nullptr);
    ~ptdlgcoldpoints();

    //新窗口非托管模式
    void prepare_date();

private slots:
    //开始选择按钮
    void on_pushButtonSelectColdPointsManually_clicked();

    //结束选择按钮
    void on_pushButtonFinishSelectColdPointsManually_clicked();

    //自动选择按钮
    void on_pushButtonSelectColdPointsAutoly_clicked();

    //添加冷却点
    void on_pushButtonDeleteColdPoints_clicked();

    void on_buttonBox_accepted();

private:
    Ui::ptdlgcoldpoints *ui;

private:
    QList<ncSolidData*> m_processedNcFile;

public:
    std::shared_ptr<ncFileData> m_ncFile;
};

#endif // PTDLGCOLDPOINTS_H
