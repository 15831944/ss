#include "rs_actioncompensate.h"
#include "ui_rs_actioncompensate.h"
#include<QDebug>
#include<QComboBox>
#include<QButtonGroup>
rs_actioncompensate::rs_actioncompensate(RS_EntityContainer &container, RS_GraphicView &graphicView,QWidget *parent) :
    QDialog(parent),
    ui(new Ui::rs_actioncompensate)
{

    ui->setupUi(this);
    connect2Db();
    init();
    QPalette palette;
    palette.setColor(QPalette::Background, QColor(255, 255, 255));
    ui->label_3->setAutoFillBackground(true);  //一定要这句，否则不行
    ui->label_3->setPalette(palette);
    QPixmap background(":/icons/buchangzhijiao.png");//    :/icons/buchangzhijiao.png  :/icons/gefengbuchang.png
    ui->label_8->setPixmap(background);
    m = new mrueditor();
    ui->comboBox->setCurrentIndex(-1);
    ui->radioButton->setChecked(true);
    QButtonGroup *group = new QButtonGroup(this);
    group->addButton(ui->radioButton,0);
    group->addButton(ui->radioButton_2,1);
    connect(group,SIGNAL(buttonClicked(int)),this,SLOT(buttonClickedSlot(int)));
    connect(ui->comboBox,SIGNAL(currentIndexChanged(int)),this,SLOT(on_comboBox_currentIndexChanged1(int)));
}

rs_actioncompensate::~rs_actioncompensate(){
    delete ui;
}

void rs_actioncompensate::connect2Db()
{
    //通过传入制定的驱动名称获得数据库连接对象
    database = QSqlDatabase::addDatabase("QSQLITE");
    //设置数据库文件的名称（针对SQLite而言）
    database.setDatabaseName("edit_info.db");
    //设置连接的用户名和密码
    database.setUserName("admin");
    database.setPassword("admin");
    //打开数据库
    if(database.open("admin","admin"))
    {
        qDebug() << "open success!!!";
    }
}

void rs_actioncompensate::init()
{
    ui->comboBox->clear();
    QSqlQuery sq;
    QString sql = "select * from edit;";
    sq.exec(sql);
    while(sq.next()){
        ui->comboBox->addItem(sq.value(0).toString());
    }
    ui->comboBox->addItem("编辑...");
    ui->comboBox->setCurrentIndex(-1);
}

void rs_actioncompensate::on_comboBox_currentIndexChanged1(int index)
{
    QSqlQuery sq;
    if(ui->comboBox->currentText()=="编辑..."){
        m->initdb();
        m->exec();
        disconnect(ui->comboBox,SIGNAL(currentIndexChanged(int)),this,SLOT(on_comboBox_currentIndexChanged1(int)));
        init();
        connect(ui->comboBox,SIGNAL(currentIndexChanged(int)),this,SLOT(on_comboBox_currentIndexChanged1(int)));
    }else{
        if(index){
            sq.prepare("select * from edit limit :value,:value;");
            sq.bindValue(":value",index);
        }else{
            sq.prepare("select*from edit;");
        }
        sq.exec();
        sq.next();
        qDebug()<<sq.value(0)<<sq.value(1)<<sq.value(2);
        ui->lineEdit->setText(sq.value(1).toString());
        ui->lineEdit_2->setText(sq.value(2).toString());
    }
}

void rs_actioncompensate::buttonClickedSlot(int id)
{
    if(id == 0){
        QPixmap background(":/icons/buchangzhijiao.png");
        ui->label_8->setPixmap(background);
    }else{
        QPixmap background(":/icons/gefengbuchang.png");
        ui->label_8->setPixmap(background);
    }
}


