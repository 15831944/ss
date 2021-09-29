#ifndef PTDLGGRIDELINESET_H
#define PTDLGGRIDELINESET_H

#include <QDialog>

#include "Qlist.h"
#include "sinsun/xstest/ncfile.h"

struct guidelineparameter
{
    guidelineparameter(int val0=0,double val1=0,double val2=0,double val3=-1){m_type=val0;m_lineLength=val1;m_Angle=val2;m_circleRadius=val3;};
    int m_type=0;//0表示无，1表示直线，2表示圆弧，3表示直线+圆弧
    double m_lineLength=0;//引线长度
    double m_Angle=0;//进入角度
    double m_circleRadius=-1;//-1表示没有圆
};

namespace Ui {
class ptdlggridelineset;
}

class ptdlggridelineset : public QDialog
{
    Q_OBJECT

public:
    explicit ptdlggridelineset(QWidget *parent = nullptr);
    ~ptdlggridelineset();
    int totalheight;

private slots:
    void on_buttonBox_accepted();

    void on_comboBoxInLineType_currentIndexChanged(int index);

    void on_comboBoxOutLineType_currentIndexChanged(int index);

    void button1Slot();
    void button2Slot();
    void button3Slot();
    void button4Slot();
//    void accept() override;
//    void reject() override;

private:
    //准备数据
    void prepare_date();

    //生成引线
    QList<QList<RS_Entity*>> generate_guide_lines(const guidelineparameter& val);

private:
    Ui::ptdlggridelineset *ui;

private:
    //对应待处理文件,i对应要修改段的位置
    QList<ncSolidData*> m_processedNcFile;

    //引出线
    QList<ncSolidData*> m_processedNcFileNext;

public:

    std::shared_ptr<ncFileData> m_ncFile;

    //调整阴切
    void set_yin_cut();

    //调整阳切
    void set_yang_cut();
};

#endif // PTDLGGRIDELINESET_H
