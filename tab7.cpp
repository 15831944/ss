#include "tab7.h"
#include "ui_tab7.h"

Tab7::Tab7(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Tab7)
{
    ui->setupUi(this);
    ui->groupBox->setObjectName("groupBox1");
    ui->groupBox->setStyleSheet("QGroupBox#groupBox1{border:none;border-right: 1px solid black;background-color:rgb(48,48,48);}");
    ui->groupBox_5->setObjectName("groupBox5");
    ui->groupBox_5->setStyleSheet("QGroupBox#groupBox5{border:none;border-right: 1px solid black;border-left: 1px solid black;background-color:rgb(48,48,48);}");
    ui->label_7->setStyleSheet("background-color:rgb(56,56,56);color:white;border-right: 1px solid black;");
    button1 = ui->toolButton_52;
    button2 = ui->toolButton_53;
    button3 = ui->toolButton_54;
    button4 = ui->toolButton_55;
}

Tab7::~Tab7()
{
    delete ui;
}
