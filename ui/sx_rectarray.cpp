//#include "sx_rectarray.h"
#include "ui_sx_rectarray.h"
#include "sx_rectanglearray.h"


sx_rectArray::sx_rectArray(RS_EntityContainer &container, RS_GraphicView &graphicView,QWidget *parent)
    :QDialog(parent)
    ,ui(new Ui::sx_rectArray)
{
    ui->setupUi(this);
    setWindowFlags(Qt::FramelessWindowHint);

    setFixedSize(width(),height());
    QFont font("Microsoft YaHei", 12, 78);
    ui->label->setFont(font);

    QIntValidator *intValidator = new QIntValidator;
    intValidator->setRange(1, 999999999);
    ui->lineEdit->setValidator(intValidator);
    ui->lineEdit_2->setValidator(intValidator);
    QDoubleValidator *doubleValidator = new QDoubleValidator;
    doubleValidator->setBottom(0);
    doubleValidator->setTop(9999);
    doubleValidator->setDecimals(2);
    ui->lineEdit_3->setValidator(doubleValidator);
    ui->lineEdit_4->setValidator(doubleValidator);
    ui->lineEdit_5->setValidator(doubleValidator);
    ui->lineEdit_6->setValidator(doubleValidator);

    ui->buttonSure->setStyleSheet("QPushButton{background:#4e4e4e;  \
                                  color:#00ffff;    \
                                  border:0;}"   \
                    "QPushButton:pressed{background:#393939;}");
    ui->buttonCancel->setStyleSheet("QPushButton{background:#4e4e4e;  \
                              color:#00ffff;    \
                              border:0;}"   \
                "QPushButton:pressed{background:#393939;}");
    connect(ui->buttonSure,SIGNAL(clicked()),this,SLOT(slotArrayNumber()));

    connect(ui->buttonCancel,SIGNAL(clicked()),this,SLOT(slotCloseWindow()));
    connect(ui->radioButton_2,SIGNAL(clicked()),this,SLOT(slotSpacing()));
    connect(ui->radioButton,SIGNAL(clicked()),this,SLOT(slotDeviation()));
}

void sx_rectArray::slotArrayNumber()
{
  if((ui->lineEdit->text() == "")||(ui->lineEdit_2->text() == "")||(ui->lineEdit_3->text() == "")||(ui->lineEdit_4->text() == ""))
  {
      QMessageBox::warning(this,"警告","输入内容不能为空!");
      return;
  }
  else if(!ui->lineEdit->text().contains(QRegExp("^\\d+$"))||!ui->lineEdit_2->text().contains(QRegExp("^\\d+$")))
  {
      QMessageBox::warning(this,"警告","输入内容不正确,请重新输入!");
      return;
  }
//  if(!ui->lineEdit_3->text().contains(QRegExp("^\\d+$"))||!ui->lineEdit_4->text().contains(QRegExp("^\\d+$")))
//  {

//  }

  if(ui->lineEdit->text() == "0")
  {
    QMessageBox::warning(this,"警告","最少为1行/1列!");
    ui->lineEdit->setText("1");
  }
  if(ui->lineEdit_2->text() == "0")
  {
    QMessageBox::warning(this,"警告","最少为1行/1列!");
    ui->lineEdit_2->setText("1");
  }
  if(!((ui->lineEdit->text().toLong() > 0)&&(ui->lineEdit->text().toLong() < 100)))
  {
    QMessageBox::warning(this,"警告","最多为100行/100列!");
    ui->lineEdit->setText("100");
  }
  if(!((ui->lineEdit_2->text().toLong() > 0)&&(ui->lineEdit_2->text().toLong() < 100)))
  {
    QMessageBox::warning(this,"警告","最多为100行/100列!");
    ui->lineEdit_2->setText("100");
  }
  if(!((ui->lineEdit_3->text().toLong() > 0)&&(ui->lineEdit_3->text().toLong() < 9999)))
  {
    QMessageBox::warning(this,"警告","最大间距为9999!");
    ui->lineEdit_3->setText("9999");
  }
  if(!((ui->lineEdit_4->text().toLong() > 0)&&(ui->lineEdit_4->text().toLong() < 9999)))
  {
    QMessageBox::warning(this,"警告","最大间距为9999!");
    ui->lineEdit_4->setText("9999");
  }

    emit rectArrayDataSignal(ui->lineEdit->text().toInt(),
                             ui->lineEdit_2->text().toInt(),
                             ui->radioButton->isChecked(),
                             ui->radioButton_2->isChecked(),
                             ui->lineEdit_3->text().toDouble(),
                             ui->lineEdit_4->text().toDouble(),
                             ui->radioButton_3->isChecked(),
                             ui->radioButton_4->isChecked(),
                             ui->radioButton_5->isChecked(),
                             ui->radioButton_6->isChecked(),
                             ui->lineEdit_5->text().toDouble(),
                             ui->lineEdit_6->text().toDouble());
  close();
}


void sx_rectArray::slotSpacing()
{
    if(ui->radioButton_2->isChecked())
    {
        ui->stackedWidget->setCurrentIndex(0);
    }
}

void sx_rectArray::slotDeviation()
{
    if(ui->radioButton->isChecked())
    {
        ui->stackedWidget->setCurrentIndex(1);
    }
}

void sx_rectArray::slotCloseWindow()
{
    emit closeWindowSignal(true);
}

void sx_rectArray::closeEvent(QCloseEvent *event)
{
    emit closeWindowSignal(true);
}

sx_rectArray::~sx_rectArray()
{
    delete ui;
}


