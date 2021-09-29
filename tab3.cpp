#include "tab3.h"
#include "ui_tab3.h"

Tab3::Tab3(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Tab3)
{
    ui->setupUi(this);

    ui->groupBox->setObjectName("groupBox1");
    ui->groupBox->setStyleSheet("QGroupBox#groupBox1{border:none;border-right: 1px solid black;background-color:rgb(48,48,48);}");
    ui->groupBox_2->setObjectName("groupBox2");
    ui->groupBox_2->setStyleSheet("QGroupBox#groupBox2{border:none;border-right: 1px solid black;border-left: 1px solid black;background-color:rgb(48,48,48);}");
    ui->groupBox_3->setObjectName("groupBox3");
    ui->groupBox_3->setStyleSheet("QGroupBox#groupBox3{border:none;border-right: 1px solid black;border-left: 1px solid black;background-color:rgb(48,48,48);}");
    ui->groupBox_4->setObjectName("groupBox4");
    ui->groupBox_4->setStyleSheet("QGroupBox#groupBox4{border:none;border-right: 1px solid black;border-left: 1px solid black;background-color:rgb(48,48,48);}");
    ui->groupBox_5->setObjectName("groupBox5");
    ui->groupBox_5->setStyleSheet("QGroupBox#groupBox5{border:none;border-right: 1px solid black;border-left: 1px solid black;background-color:rgb(48,48,48);}");
    ui->label_7->setStyleSheet("background-color:rgb(56,56,56);color:white;border-right: 1px solid black;");
    ui->label_6->setStyleSheet("background-color:rgb(56,56,56);color:white;border-right: 1px solid black;border-left: 1px solid black;");
    ui->label_8->setStyleSheet("background-color:rgb(56,56,56);color:white;border-right: 1px solid black;border-left: 1px solid black;");
    ui->label_5->setStyleSheet("background-color:rgb(56,56,56);color:white;border-right: 1px solid black;border-left: 1px solid black;");
    button1 = ui->toolButton_52;
    button2 = ui->toolButton_53;
    button3 = ui->toolButton_54;
    button4 = ui->toolButton_55;
    button5 = ui->toolButton_56;
    button6 = ui->toolButton_57;
    button7 = ui->toolButton_58;
    button8 = ui->toolButton_59;
    button9 = ui->toolButton_60;
    button10 = ui->toolButton_61;
    button11 = ui->toolButton_62;
    button12 = ui->toolButton_63;
    button13 = ui->toolButton_64;
    button14 = ui->toolButton_65;
    button15 = ui->toolButton_66;
    button16 = ui->toolButton_67;
}

Tab3::~Tab3()
{
    delete ui;
}
