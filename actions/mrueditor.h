#ifndef MRUEDITOR_H
#define MRUEDITOR_H

#include <QDialog>
#include <QSqlDatabase>
#include <QSqlQuery>
#include <QSqlError>
namespace Ui {
class mrueditor;
}

class mrueditor : public QDialog
{
    Q_OBJECT

public:
    explicit mrueditor(QWidget *parent = nullptr);
    ~mrueditor();
    void accept() override;
    QSqlDatabase database; //数据库连接对象
    void connect2Db(); //连接并打开数据库
  //  void createTable(); //创建表
    void initdb();
private slots:
    void on_add_clicked();
    void on_deleteitem_clicked();
    void on_clear_clicked();
    void cellChangeSlot(int row,int column);

private:
    Ui::mrueditor *ui;
};

#endif // MRUEDITOR_H
