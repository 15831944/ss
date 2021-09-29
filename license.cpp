#include "license.h"
#include "ui_license.h"
#include"textfileviewer.h"
#include<QDialogButtonBox>
#include<QPushButton>
#include <QFile>
#include <QTextStream>
#include <QPlainTextEdit>
#include <QListWidget>
License::License(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::License)
{
    ui->setupUi(this);

this->setWindowFlag(Qt::FramelessWindowHint);

    ui->plainTextEdit->setReadOnly(true);
    connect(ui->listWidget, &QListWidget::itemClicked, this, &License::loadFile);
    addFile("readme", ":/readme.md");
    addFile("GPLv2", ":/gpl-2.0.txt");
    setFile("readme");
    ui->plainTextEdit->setVerticalScrollBar(ui->verticalScrollBar);
    ui->buttonBox->button(QDialogButtonBox::Ok)->setStyleSheet("background-color:#4E4E4E;color:#0ff;width:80px;height:25px;font-size:18px");
    ui->buttonBox->button(QDialogButtonBox::Cancel)->hide();
    ui->buttonBox->button(QDialogButtonBox::Ok)->setText("确定");
    connect(ui->buttonBox->button(QDialogButtonBox::Ok),SIGNAL(clicked()),this,SLOT(buttonClick()));
    ui->verticalScrollBar->setStyleSheet("QScrollBar:vertical{margin:0;width:10px;border-width:0px}\
                                QScrollBar::sub-page:vertical{background-color:#707070;border:0;}\
                                QScrollBar::add-page:vertical{background-color:#707070;border:0;}\
                                 QScrollBar::handle:vertical{margin:0;border:0;background:#292929}\
                                 QScrollBar::sub-line:vertical{image:none;}\
                                 QScrollBar::add-line:vertical{image:none;}");

}

License::~License()
{
    delete ui;
}

bool License::addFile(QString name, QString path)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        return false;

    QTextStream txt_stream(&file);
    auto txt = txt_stream.readAll();

    auto item = new QListWidgetItem(name, ui->listWidget);
    item->setWhatsThis(txt);
    return true;
}

void License::setFile(QString name)
{
    auto item = ui->listWidget->findItems(name, Qt::MatchExactly)[0];
    ui->listWidget->setCurrentItem(item);
    loadFile(item);
}

void License::loadFile(QListWidgetItem *item)
{
    ui->plainTextEdit->setPlainText(item->whatsThis());
}

void License::buttonClick()
{
    this->close();
}
