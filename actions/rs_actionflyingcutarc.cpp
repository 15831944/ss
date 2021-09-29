#include "rs_actionflyingcutarc.h"
#include "ui_rs_actionflyingcutarc.h"

rs_actionflyingcutarc::rs_actionflyingcutarc(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::rs_actionflyingcutarc)
{
    ui->setupUi(this);
    QPalette palette;
    palette.setColor(QPalette::Background, QColor(255, 255, 255));
    ui->label->setAutoFillBackground(true);  //一定要这句，否则不行
    ui->label->setPalette(palette);
    QPixmap background1(":/icons/julipiancha.png");
    ui->label_5->setPixmap(background1);
}

rs_actionflyingcutarc::~rs_actionflyingcutarc()
{
    delete ui;
}
