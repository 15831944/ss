#include "sx_interactivearray.h"
#include "ui_sx_interactivearray.h"

sx_interactiveArray::sx_interactiveArray(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::sx_interactiveArray)
{
    ui->setupUi(this);
    setWindowTitle("交互式阵列");
    setWindowFlags(Qt::FramelessWindowHint);
    setFixedSize(width(),height());
//    QFont font("Microsoft YaHei", 12, 78);
//    ui->label_2->setFont(font);

//    ui->label->setStyleSheet("background-color:white");
    QPixmap map(":/main/hang.png");
    int w = map.width();
    int h = map.height();
//    ui->label_3->resize(w,h);
//    ui->label_3->setPixmap(map);

    QPixmap map2(":/main/lie.png");
    int w2 = map.width();
    int h2 = map.height();
//    ui->label_5->resize(w2,h2);
//    ui->label_5->setPixmap(map2);

    QDoubleValidator *doubleValidator = new QDoubleValidator;
    doubleValidator->setBottom(0);
    doubleValidator->setTop(9999);
    doubleValidator->setDecimals(2);
    ui->lineEdit->setValidator(doubleValidator);
    ui->lineEdit_2->setValidator(doubleValidator);


    ui->pushButton->setStyleSheet("QPushButton{background:#4e4e4e;  \
                                  color:#00ffff;    \
                                  border:0;}"   \
                    "QPushButton:pressed{background:#393939;}");
    ui->pushButton_2->setStyleSheet("QPushButton{background:#4e4e4e;  \
                              color:#00ffff;    \
                              border:0;}"   \
                "QPushButton:pressed{background:#393939;}");



    connect(ui->pushButton,SIGNAL(clicked()),this,SLOT(slotClickOk()));
    connect(ui->pushButton_2,SIGNAL(clicked()),this,SLOT(slotCloseWindow()));
}


void sx_interactiveArray::slotClickOk()
{
    if((ui->lineEdit->text() == "")||(ui->lineEdit_2->text() == ""))
    {
        QMessageBox::warning(this,"警告","输入内容不能为空!");
        return;
    }
    else if(!ui->lineEdit->text().contains(QRegExp("^\\d+$"))||!ui->lineEdit_2->text().contains(QRegExp("^\\d+$")))
    {
        QMessageBox::warning(this,"警告","输入内容不正确,请重新输入!");
        return;
    }
    if(ui->lineEdit->text() == "0")
    {
      QMessageBox::warning(this,"警告","最少为1行/1列!");
      ui->lineEdit->setText("1");
    }
    else if(ui->lineEdit_2->text() == "0")
    {
      QMessageBox::warning(this,"警告","最少为1行/1列!");
      ui->lineEdit_2->setText("1");
    }
    if(!((ui->lineEdit->text().toLong() > 0)&&(ui->lineEdit->text().toLong() < 10000)))
    {
      QMessageBox::warning(this,"警告","最大间距为9999!");
      ui->lineEdit->setText("9999");
      if(!((ui->lineEdit_2->text().toLong() > 0)&&(ui->lineEdit_2->text().toLong() < 10000)))
          {
            ui->lineEdit_2->setText("9999");
          }
    }
    else if(!((ui->lineEdit_2->text().toLong() > 0)&&(ui->lineEdit_2->text().toLong() < 10000)))
    {
      QMessageBox::warning(this,"警告","最大间距为9999!");
      ui->lineEdit_2->setText("9999");
      if(!((ui->lineEdit->text().toLong() > 0)&&(ui->lineEdit->text().toLong() < 10000)))
      {
          ui->lineEdit->setText("9999");
      }
    }
    emit sendDataSignal(true,ui->checkBox->isChecked(),ui->lineEdit->text().toDouble(),ui->lineEdit_2->text().toDouble());

    hide();   //调用close之后鼠标事件失效
//    close();
}

sx_interactiveArray::~sx_interactiveArray()
{
    delete ui;
}

void sx_interactiveArray::slotCloseWindow()
{
    emit closeWindowSignal(true);
}


void sx_interactiveArray::closeEvent(QCloseEvent *event)
{
    emit closeWindowSignal(true);
}
