#include "tab5.h"
#include "ui_tab5.h"

Tab5::Tab5(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Tab5)
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


    ui->groupBox_6->setObjectName("groupBox6");
    ui->groupBox_6->setStyleSheet("QGroupBox#groupBox6{border:none;border-right: 1px solid black;border-left: 1px solid black;background-color:rgb(48,48,48);}");
    ui->groupBox_7->setObjectName("groupBox7");
    ui->groupBox_7->setStyleSheet("QGroupBox#groupBox7{border:none;border-right: 1px solid black;border-left: 1px solid black;background-color:rgb(48,48,48);}");
    ui->groupBox_8->setObjectName("groupBox8");
    ui->groupBox_8->setStyleSheet("QGroupBox#groupBox8{border:none;border-right: 1px solid black;border-left: 1px solid black;background-color:rgb(48,48,48);}");
    ui->groupBox_9->setObjectName("groupBox9");
    ui->groupBox_9->setStyleSheet("QGroupBox#groupBox9{border:none;border-right: 1px solid black;border-left: 1px solid black;background-color:rgb(48,48,48);}");
    ui->groupBox_10->setObjectName("groupBox10");
    ui->groupBox_10->setStyleSheet("QGroupBox#groupBox10{border:none;border-right: 1px solid black;border-left: 1px solid black;background-color:rgb(48,48,48);}");

    ui->label_7->setStyleSheet("background-color:rgb(56,56,56);color:white;border-right: 1px solid black;");
    ui->label_6->setStyleSheet("background-color:rgb(56,56,56);color:white;border-right: 1px solid black;border-left: 1px solid black;");
    ui->label_8->setStyleSheet("background-color:rgb(56,56,56);color:white;border-right: 1px solid black;border-left: 1px solid black;");
    ui->label_5->setStyleSheet("background-color:rgb(56,56,56);color:white;border-right: 1px solid black;border-left: 1px solid black;");
    ui->label_9->setStyleSheet("background-color:rgb(56,56,56);color:white;border-right: 1px solid black;border-left: 1px solid black;");
    ui->label_10->setStyleSheet("background-color:rgb(56,56,56);color:white;border-right: 1px solid black;border-left: 1px solid black;");
    ui->label_11->setStyleSheet("background-color:rgb(56,56,56);color:white;border-right: 1px solid black;border-left: 1px solid black;");
    ui->label_12->setStyleSheet("background-color:rgb(56,56,56);color:white;border-right: 1px solid black;border-left: 1px solid black;");
    ui->label_14->setStyleSheet("background-color:rgb(56,56,56);color:white;border-right: 1px solid black;border-left: 1px solid black;");
    button1 = ui->toolButton_52;
    button2 = ui->toolButton_53;
    button3 = ui->toolButton_54;
    button4 = ui->toolButton_55;
    button5 = ui->toolButton_63;
    button6 = ui->toolButton_64;

    button7 = ui->toolButton_68;
    button8 = ui->toolButton_69;
    button9 = ui->toolButton_70;
    button10 = ui->toolButton_71;
    button11 = ui->toolButton_72;
    button12 = ui->toolButton_73;
    button13 = ui->toolButton_74;

    button14 = ui->toolButton_65;
    button15 = ui->toolButton_66;
    button16 = ui->toolButton_67;

    button17 = ui->toolButton_75;
    button18 = ui->toolButton_76;
    button19 = ui->toolButton_77;
    button20 = ui->toolButton_78;
    button21 = ui->toolButton_79;
    button22 = ui->toolButton_80;
    button23 = ui->toolButton_81;
    button24 = ui->toolButton_82;

    button25 = ui->toolButton_90;
    button26 = ui->toolButton_92;
    button27 = ui->toolButton_91;
    button28 = ui->toolButton_93;

}

Tab5::~Tab5()
{
    delete ui;
}
