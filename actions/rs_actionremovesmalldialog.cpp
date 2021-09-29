#include "rs_actionremovesmall.h"
#include "ui_rs_actionremovesmalldialog.h"
#include<QDebug>
rs_actionremovesmalldialog::rs_actionremovesmalldialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::rs_actionremovesmalldialog)
{
    ui->setupUi(this);
    QPalette palette;
    palette.setColor(QPalette::Background, QColor(255, 255, 255));
    ui->label->setAutoFillBackground(true);  //一定要这句，否则不行
    ui->label->setPalette(palette);
    connect(ui->lineEdit,SIGNAL(textChanged(QString)),this,SLOT(lineeditSlot(QString)));
    connect(ui->pushButton,SIGNAL(clicked()),this,SLOT(accept1()));
    connect(ui->pushButton_2,SIGNAL(clicked()),this,SLOT(reject1()));
}

rs_actionremovesmalldialog::~rs_actionremovesmalldialog()
{
    delete ui;
}

void rs_actionremovesmalldialog::accept1()
{
    emit myRemoveSmallSignal(ui->lineEdit->text().toDouble(),1);
    this->close();
}

void rs_actionremovesmalldialog::reject1()
{
    emit myRemoveSmallSignal(ui->lineEdit->text().toDouble(),0);
    this->close();
}

void rs_actionremovesmalldialog::closeEvent(QCloseEvent *event)
{
    emit closeWindowSignal(true);
}

void rs_actionremovesmalldialog::lineeditSlot(QString text)
{
    if(text.toDouble()>100)
        ui->lineEdit->setText("100");
    if(text.toDouble()<0.001)
        ui->lineEdit->setText("0.001");
}
