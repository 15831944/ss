#include "rs_actionsetwiredialog.h"
#include "ui_rs_actionsetwiredialog.h"

rs_actionsetwiredialog::rs_actionsetwiredialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::rs_actionsetwiredialog)
{
    ui->setupUi(this);
    QPalette palette;
    palette.setColor(QPalette::Background, QColor(255, 255, 255));
    ui->label->setAutoFillBackground(true);  //一定要这句，否则不行
    ui->label->setPalette(palette);
    ui->label_7->setEnabled(false);
    ui->label_8->setEnabled(false);
    ui->lineEdit_3->setEnabled(false);
    ui->lineEdit_4->setEnabled(false);
    ui->label_12->setEnabled(false);
    ui->lineEdit_7->setEnabled(false);
    ui->radioButton->setChecked(true);
    connect(ui->comboBox,SIGNAL(currentIndexChanged(int)),this,SLOT(comboboxIntroduceSlot(int)));
    connect(ui->comboBox_2,SIGNAL(currentIndexChanged(int)),this,SLOT(comboboxExtractionSlot(int)));
    connect(ui->checkBox,SIGNAL(stateChanged(int)),this,SLOT(checkBoxSlot(int)));
    radiogroup = new QButtonGroup(this);
    radiogroup->addButton(ui->radioButton,1);
    radiogroup->addButton(ui->radioButton_2,2);
    radiogroup->addButton(ui->radioButton_3,3);
    connect(radiogroup,SIGNAL(buttonClicked(int)),this,SLOT(radioSlot(int)));
    connect(ui->checkBox_4,SIGNAL(stateChanged(int)),this,SLOT(checkSlot1(int)));
    connect(ui->checkBox_5,SIGNAL(stateChanged(int)),this,SLOT(checkSlot2(int)));
    connect(ui->checkBox_7,SIGNAL(stateChanged(int)),this,SLOT(checkSlot3(int)));
    connect(ui->checkBox_9,SIGNAL(stateChanged(int)),this,SLOT(checkSlot4(int)));
}

rs_actionsetwiredialog::~rs_actionsetwiredialog()
{
    delete ui;
}

void rs_actionsetwiredialog::comboboxIntroduceSlot(int index)
{
    switch(index){
    case 0:
        ui->label_7->setEnabled(false);
        ui->label_8->setEnabled(false);
        ui->lineEdit_3->setEnabled(false);
        ui->lineEdit_4->setEnabled(false);
        ui->checkBox->setEnabled(true);
        break;
    case 1:
        ui->label_7->setEnabled(false);
        ui->label_8->setEnabled(false);
        ui->checkBox->setEnabled(true);
        ui->lineEdit_3->setEnabled(false);
        ui->lineEdit_4->setEnabled(false);
        break;
    case 2:
        ui->label_7->setEnabled(false);
        ui->label_8->setEnabled(false);
        ui->lineEdit_3->setEnabled(false);
        ui->lineEdit_4->setEnabled(false);
        ui->checkBox->setChecked(false);
        ui->checkBox->setEnabled(false);
        break;
    case 3:
        ui->label_8->setEnabled(false);
        ui->lineEdit_4->setEnabled(false);
        ui->label_7->setEnabled(true);
        ui->lineEdit_3->setEnabled(true);
        ui->checkBox->setEnabled(true);
        break;
    default:break;
    }
}

void rs_actionsetwiredialog::comboboxExtractionSlot(int index)
{
    switch(index){
    case 0:
        ui->label_12->setEnabled(false);
        ui->lineEdit_7->setEnabled(false);
        break;
    case 1:
        ui->label_12->setEnabled(false);
        ui->lineEdit_7->setEnabled(false);
        break;
    case 2:
        ui->label_12->setEnabled(false);
        ui->lineEdit_7->setEnabled(false);
        break;
    case 3:
        ui->label_12->setEnabled(true);
        ui->lineEdit_7->setEnabled(true);
        break;
    default:break;
    }
}

void rs_actionsetwiredialog::checkBoxSlot(int state)
{
    if(!state){
        ui->label_8->setEnabled(false);
        ui->lineEdit_4->setEnabled(false);
    }else{
        ui->label_8->setEnabled(true);
        ui->lineEdit_4->setEnabled(true);
    }
}

void rs_actionsetwiredialog::radioSlot(int id)
{
    switch(id){
    case 1:
        ui->checkBox_4->setEnabled(true);
        ui->checkBox_5->setEnabled(true);
        ui->lineEdit_8->setEnabled(false);
        break;
    case 2:
        ui->checkBox_4->setEnabled(false);
        ui->checkBox_5->setEnabled(false);
        ui->lineEdit_8->setEnabled(true);
        break;
    case 3:
        ui->checkBox_4->setEnabled(false);
        ui->checkBox_5->setEnabled(false);
        ui->lineEdit_8->setEnabled(false);
        break;
    case 4:
        ui->checkBox_4->setEnabled(false);
        ui->checkBox_5->setEnabled(false);
        ui->lineEdit_8->setEnabled(false);
        break;
    default:break;
    }
}

void rs_actionsetwiredialog::checkSlot1(int state)
{
    if(state)
        ui->checkBox_5->setChecked(0);
}

void rs_actionsetwiredialog::checkSlot2(int state)
{
    if(state)
        ui->checkBox_4->setChecked(0);
}

void rs_actionsetwiredialog::checkSlot3(int state)
{
    if(state)
        ui->checkBox_9->setChecked(0);
}

void rs_actionsetwiredialog::checkSlot4(int state)
{
    if(state)
        ui->checkBox_7->setChecked(0);
}
