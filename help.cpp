#include "help.h"
#include "ui_help.h"
#include<QDialogButtonBox>
#include<QPushButton>
#include<QDesktopServices>
#include<QUrl>
help::help(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::help)
{
    ui->setupUi(this);
    ui->groupBox->setStyleSheet("border:0px");
    ui->label->setStyleSheet("color:#0ff;font-size:16px");
    ui->label_2->setStyleSheet("color:#0ff;font-size:16px");
    ui->label_3->setStyleSheet("color:#0ff;font-size:16px");
    ui->label_4->setStyleSheet("color:#0ff;font-size:16px");
    ui->label_5->setStyleSheet("color:#0ff;font-size:16px");
    ui->label_6->setStyleSheet("color:#0ff;font-size:16px");
    ui->label_7->setStyleSheet("color:#0ff;font-size:16px");
    this->setWindowFlag(Qt::FramelessWindowHint);
    ui->buttonBox->button(QDialogButtonBox::Ok)->setStyleSheet("background-color:#4E4E4E;color:#0ff;width:80px;height:25px;font-size:18px");
//    ui->buttonBox->button(QDialogButtonBox::Cancel)->setStyleSheet("background-color:#4E4E4E;color:#0ff;width:80px;height:25px;font-size:18px");
//    ui->buttonBox->button(QDialogButtonBox::Cancel)->setText("取消");
    ui->buttonBox->button(QDialogButtonBox::Cancel)->hide();
    ui->buttonBox->button(QDialogButtonBox::Ok)->setText("确定");
    connect(ui->buttonBox->button(QDialogButtonBox::Ok),SIGNAL(clicked()),this,SLOT(buttonClick()));

    ui->label->setText("<a href = 'https://dokuwiki.librecad.org/' style = 'color:#0ff;text-decoration: none'>维基</a>");
    ui->label_2->setText("<a href = 'https://librecad.readthedocs.io/' style = 'color:#0ff;text-decoration: none'>用户手册</a>");
    ui->label_3->setText("<a href = 'https://wiki.librecad.org/index.php/Commands' style = 'color:#0ff;text-decoration: none'>命令</a>");
    ui->label_4->setText("<a href = 'https://github.com/LibreCAD/LibreCAD/wiki/Style-Sheets' style = 'color:#0ff;text-decoration: none'>样式表</a>");
    ui->label_5->setText("<a href = 'https://github.com/LibreCAD/LibreCAD/wiki/Widgets' style = 'color:#0ff;text-decoration: none'>窗体</a>");
    ui->label_6->setText("<a href = 'https://forum.librecad.org/' style = 'color:#0ff;text-decoration: none'>论坛</a>");
    ui->label_7->setText("<a href = 'https://github.com/LibreCAD/LibreCAD/releases' style = 'color:#0ff;text-decoration: none'>Release Information</a>");

    ui->label->setOpenExternalLinks(true);
    ui->label_2->setOpenExternalLinks(true);
    ui->label_3->setOpenExternalLinks(true);
    ui->label_4->setOpenExternalLinks(true);
    ui->label_5->setOpenExternalLinks(true);
    ui->label_6->setOpenExternalLinks(true);
    ui->label_7->setOpenExternalLinks(true);
}

help::~help()
{
    delete ui;
}



void help::buttonClick()
{
    this->close();
}

