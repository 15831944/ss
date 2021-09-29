#include "rs_actionflyingcut.h"
#include "ui_rs_actionflyingcut.h"

rs_actionflyingcut::rs_actionflyingcut(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::rs_actionflyingcut)
{
    ui->setupUi(this);
    QPalette palette;
    palette.setColor(QPalette::Background, QColor(255, 255, 255));
    ui->label->setAutoFillBackground(true);  //一定要这句，否则不行
    ui->label->setPalette(palette);
    ui->label_7->setEnabled(false);
    ui->lineEdit_3->setEnabled(false);
    QPixmap background1(":/icons/julipiancha.png");
    ui->label_8->setPixmap(background1);
    QPixmap background2(":/icons/zuidajuli.png");
    ui->label_9->setPixmap(background2);
    QPixmap background3(":/icons/feixingxianchangdu.png");
    ui->label_10->setPixmap(background3);
    connect(ui->checkBox,SIGNAL(stateChanged(int)),this,SLOT(checkboxSlot(int)));
}

rs_actionflyingcut::~rs_actionflyingcut()
{
    delete ui;
}

void rs_actionflyingcut::checkboxSlot(int state)
{
    if(!state){
        ui->label_7->setEnabled(false);
        ui->lineEdit_3->setEnabled(false);
    }else if(state==2){
        ui->label_7->setEnabled(true);
        ui->lineEdit_3->setEnabled(true);
    }
}
