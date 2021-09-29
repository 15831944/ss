#include "rs_actionmergenew.h"
#include "ui_rs_actionmerge.h"
#include<QDebug>
rs_actionmerge::rs_actionmerge(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::rs_actionmerge)
{
    ui->setupUi(this);
    QPalette palette;
    palette.setColor(QPalette::Background, QColor(255, 255, 255));
    ui->label->setAutoFillBackground(true);  //一定要这句，否则不行
    ui->label->setPalette(palette);
    connect(ui->pushButton,SIGNAL(clicked()),this,SLOT(okSlot()));
    connect(ui->pushButton_2,SIGNAL(clicked()),this,SLOT(cancelSlot()));

}

rs_actionmerge::~rs_actionmerge()
{
    delete ui;
}

void rs_actionmerge::okSlot()
{
    emit myMergeSignal(ui->lineEdit->text().toDouble());
    this->close();
}

void rs_actionmerge::cancelSlot()
{
    this->close();
}


void rs_actionmerge::closeEvent(QCloseEvent *event)
{
    emit closeWindowSignal(true);
}
