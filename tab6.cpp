#include "tab6.h"
#include "ui_tab6.h"

Tab6::Tab6(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Tab6)
{
    ui->setupUi(this);
    ui->groupBox->setObjectName("groupBox1");
    ui->groupBox->setStyleSheet("QGroupBox#groupBox1{border:none;border-right: 1px solid black;background-color:rgb(48,48,48);}");
    ui->groupBox_2->setObjectName("groupBox2");
    ui->groupBox_2->setStyleSheet("QGroupBox#groupBox2{border:none;border-right: 1px solid black;border-left: 1px solid black;background-color:rgb(48,48,48);}");
    ui->groupBox_5->setObjectName("groupBox5");
    ui->groupBox_5->setStyleSheet("QGroupBox#groupBox5{border:none;border-right: 1px solid black;border-left: 1px solid black;background-color:rgb(48,48,48);}");
    ui->label_7->setStyleSheet("background-color:rgb(56,56,56);color:white;border-right: 1px solid black;");
    ui->label_6->setStyleSheet("background-color:rgb(56,56,56);color:white;border-right: 1px solid black;border-left: 1px solid black;");
    button1 = ui->toolButton_52;
//    button2 = ui->toolButton_53;
    button3 = ui->toolButton_60;
    button4 = ui->toolButton_59;
    button5 = ui->toolButton_54;
    button6 = ui->toolButton_55;
    button7 = ui->toolButton_56;
    button8 = ui->toolButton_57;
    button9 = ui->toolButton_61;
    button10 = ui->toolButton_62;
    button11 = ui->toolButton_63;
}

Tab6::~Tab6()
{
    delete ui;
}
