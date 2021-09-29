#include "tab1.h"
#include "ui_tab1.h"

Tab1::Tab1(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Tab1)
{
    ui->setupUi(this);
    button1 = ui->toolButton;
    button2 = ui->toolButton_2;
    button3 = ui->toolButton_3;
    button4 = ui->toolButton_4;
    button5 = ui->toolButton_5;
    button6 = ui->toolButton_6;
    button7 = ui->toolButton_7;
    button8 = ui->toolButton_8;
    button9 = ui->toolButton_12;
    button10 = ui->toolButton_10;
    button11 = ui->toolButton_13;

    button12 = ui->toolButton_17;
    button13 = ui->toolButton_18;
    button14 = ui->toolButton_20;
    button15 = ui->toolButton_14;
    button16 = ui->toolButton_16;
    button17 = ui->toolButton_15;

    button18 = ui->toolButton_9;
    button19 = ui->toolButton_11;
    button20 = ui->toolButton_19;

    button21 = ui->toolButton_21;
    button22 = ui->toolButton_22;
    button23 = ui->toolButton_23;
    button24 = ui->toolButton_24;
    button25 = ui->toolButton_25;
    button26 = ui->toolButton_27;
    button27 = ui->toolButton_26;
    button28 = ui->toolButton_28;
    ui->groupBox->setObjectName("groupBox1");
    ui->groupBox->setStyleSheet("QGroupBox#groupBox1{border:none;border-right: 1px solid black;background-color:rgb(48,48,48);}");
    ui->groupBox_2->setObjectName("groupBox2");
    ui->groupBox_2->setStyleSheet("QGroupBox#groupBox2{border:none;border-right: 1px solid black;border-left: 1px solid black;background-color:rgb(48,48,48);}");
    ui->groupBox_3->setObjectName("groupBox3");
    ui->groupBox_3->setStyleSheet("QGroupBox#groupBox3{border:none;border-right: 1px solid black;border-left: 1px solid black;background-color:rgb(48,48,48);}");
    ui->groupBox_4->setObjectName("groupBox4");
    ui->groupBox_4->setStyleSheet("QGroupBox#groupBox4{border:none;border-right: 1px solid black;border-left: 1px solid black;background-color:rgb(48,48,48);}");
//    ui->groupBox_5->setObjectName("groupBox5");
//    ui->groupBox_5->setStyleSheet("QGroupBox#groupBox5{border:none;border-right: 1px solid black;border-left: 1px solid black;background-color:rgb(48,48,48);}");
    ui->label->setStyleSheet("background-color:rgb(56,56,56);color:white;border-right: 1px solid black;");
    ui->label_2->setStyleSheet("background-color:rgb(56,56,56);color:white;border-right: 1px solid black;border-left: 1px solid black;");
    ui->label_3->setStyleSheet("background-color:rgb(56,56,56);color:white;border-right: 1px solid black;border-left: 1px solid black;");
    ui->label_4->setStyleSheet("background-color:rgb(56,56,56);color:white;border-right: 1px solid black;border-left: 1px solid black;");
}

Tab1::~Tab1()
{
    delete ui;
}
