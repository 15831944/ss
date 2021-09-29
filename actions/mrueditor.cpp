#include "mrueditor.h"
#include "ui_mrueditor.h"
#include<QDebug>
#include<QDir>
mrueditor::mrueditor(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::mrueditor)
{
    ui->setupUi(this);
    connect2Db();
    initdb();
    QHeaderView *header = ui->tableWidget->verticalHeader();
    header->setHidden(true);// 隐藏行号
    ui->tableWidget->setSelectionMode(QAbstractItemView::SingleSelection);
    connect(ui->tableWidget,SIGNAL(cellChanged(int,int)),this,SLOT(cellChangeSlot(int,int)));
}

mrueditor::~mrueditor()
{
    delete ui;
}

void mrueditor::accept()
{
    this->close();
    QString describe1;
    double outside1;
    double within1;
    QSqlQuery sq;
    QString sql = "DELETE FROM edit";
    sq.exec(sql);
    sq.prepare("insert into edit values(:describe, :outside, :within);");
    for(int row = 0;row < ui->tableWidget->rowCount(); row++)
    {
        if(ui->tableWidget->item(row,0)->text()==""){
            ui->tableWidget->removeRow(row);
            continue;
        }
        describe1 = ui->tableWidget->item(row,0)->text();
        outside1 = ui->tableWidget->item(row,1)->text().toDouble();
        within1 = ui->tableWidget->item(row,2)->text().toDouble();
        sq.bindValue(":describe",describe1);
        sq.bindValue(":outside",outside1);
        sq.bindValue(":within",within1);
        sq.exec();
    }

}

void mrueditor::connect2Db()
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
        //createTable();

    }
}

void mrueditor::initdb()
{
    on_clear_clicked();
    QSqlQuery sq;
    QString sql = "select * from edit;";
    sq.exec(sql);
    int i = 0;
    while(sq.next()){
        ui->tableWidget->insertRow(i);
        ui->tableWidget->setItem(i,0,new QTableWidgetItem(sq.value(0).toString()));
        ui->tableWidget->setItem(i,1,new QTableWidgetItem(sq.value(1).toString()));
        ui->tableWidget->setItem(i,2,new QTableWidgetItem(sq.value(2).toString()));
        i++;
    }
}

/*void mrueditor::createTable()
{
    //拿到一个数据库操作对象
    QSqlQuery sq;
    //准备一个建表语句
    QString sql = "CREATE TABLE edit(describe TEXT,outside REAL,within REAL)";
    //执行sql语句
    if(sq.exec(sql))
    {
        qDebug() << "create table success!!!";
    }
}
*/
void mrueditor::on_add_clicked()
{
    if(!ui->tableWidget->rowCount()){
        ui->tableWidget->insertRow(ui->tableWidget->rowCount());
        ui->tableWidget->setItem(ui->tableWidget->rowCount()-1,0,new QTableWidgetItem(""));
        ui->tableWidget->setItem(ui->tableWidget->rowCount()-1,1,new QTableWidgetItem("0.005"));
        ui->tableWidget->setItem(ui->tableWidget->rowCount()-1,2,new QTableWidgetItem("0.005"));
    }else if(ui->tableWidget->item(ui->tableWidget->rowCount()-1,0)->text()!=""&&ui->tableWidget->rowCount()<8){
        ui->tableWidget->insertRow(ui->tableWidget->rowCount());
        ui->tableWidget->setItem(ui->tableWidget->rowCount()-1,0,new QTableWidgetItem(""));
        ui->tableWidget->setItem(ui->tableWidget->rowCount()-1,1,new QTableWidgetItem("0.005"));
        ui->tableWidget->setItem(ui->tableWidget->rowCount()-1,2,new QTableWidgetItem("0.005"));
    }
    ui->tableWidget->editItem(ui->tableWidget->item(ui->tableWidget->rowCount()-1,0));

}

void mrueditor::on_deleteitem_clicked()
{
    for(int row = ui->tableWidget->rowCount() - 1;row >= 0; row--)
    {
        for(int column = 0; column < 2; column++){
            if(ui->tableWidget->item(row,column)->isSelected()){
                ui->tableWidget->removeRow(row);
                return;
            }
        }

    }
    ui->tableWidget->removeRow(ui->tableWidget->rowCount()-1);
}

void mrueditor::on_clear_clicked()
{
    for(int row = ui->tableWidget->rowCount() - 1;row >= 0; row--)
    {
        ui->tableWidget->removeRow(row);
    }
}

void mrueditor::cellChangeSlot(int row, int column)
{
    if(column>0){
        if(ui->tableWidget->item(row,column)->text().toDouble()>10){
            ui->tableWidget->setItem(row,column,new QTableWidgetItem("10"));
        }else if(ui->tableWidget->item(row,column)->text().toDouble()<0.005){
            ui->tableWidget->setItem(row,column,new QTableWidgetItem("0.005"));
        }
    }
}
