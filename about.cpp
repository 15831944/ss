#include "about.h"
#include "ui_about.h"
#include<QDialogButtonBox>
#include<QPushButton>
#include<QDesktopServices>
#include<QUrl>
#include<QClipboard>
about::about(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::about)
{
    ui->setupUi(this);
    button1 = ui->pushButton;
    ui->buttonBox->button(QDialogButtonBox::Ok)->setStyleSheet("background-color:#4E4E4E;color:#0ff;width:80px;height:25px;font-size:18px");
    this->setWindowFlag(Qt::FramelessWindowHint);
    ui->buttonBox->button(QDialogButtonBox::Cancel)->hide();
    connect(ui->buttonBox->button(QDialogButtonBox::Ok),SIGNAL(clicked()),this,SLOT(buttonClick()));
    ui->label->setText("<a href = 'https://github.com/LibreCAD/LibreCAD/graphs/contributors\' style = 'color:#0ff;text-decoration: none'>贡献人员</a>");
    ui->label_5->setText("<a href = 'https://github.com/LibreCAD/LibreCAD/blob/master/LICENSE\' style = 'color:#0ff;text-decoration: none'>许可</a>");
    ui->label_2->setText("<a href = 'https://github.com/LibreCAD/LibreCAD/tree/%1\' style = 'color:#0ff;text-decoration: none'>The Code</a>");
    ui->label->setOpenExternalLinks(true);
    ui->label_2->setOpenExternalLinks(true);
    ui->label_5->setOpenExternalLinks(true);
    ui->pushButton->setStyleSheet("QPushButton{background-color:#4E4E4E;color:#0ff;font-size:18px;border:1px solid black;}"\
    "QPushButton::pressed{background-color:#303030;}");
    connect(ui->pushButton,SIGNAL(clicked()),this,SLOT(copySlot()));

}

about::~about()
{
    delete ui;
}

void about::setLabelText(QString text)
{
    ui->label_3->setText(text);
}

void about::buttonClick()
{
    this->close();
}

void about::copySlot()
{
    QClipboard *clipboard = QApplication::clipboard();   //获取系统剪贴板指针
    QString originalText = ui->label_3->text();	     //获取剪贴板上文本信息
    clipboard->setText(originalText);

}
