#ifndef RS_ACTIONCOMPENSATE_H
#define RS_ACTIONCOMPENSATE_H

#include <QDialog>
//#include<QObject>
#include"rs_entitycontainer.h"
#include <QMouseEvent>
#include"rs_entity.h"
#include"rs_modification.h"
#include "rs_vector.h"
#include "QPoint"
#include"rs_graphicview.h"
#include "rs_previewactioninterface.h"
#include "mrueditor.h"

namespace Ui {
class rs_actioncompensate;
}

class rs_actioncompensate : public QDialog
{
   // Q_OBJECT

public:
    explicit rs_actioncompensate(RS_EntityContainer &container, RS_GraphicView &graphicView,QWidget *parent = nullptr);
    ~rs_actioncompensate();
    mrueditor* m;
private:
    QSqlDatabase database; //数据库连接对象
    void connect2Db(); //连接并打开数据库
    void init();
private slots:
    void on_comboBox_currentIndexChanged1(int index);
    void buttonClickedSlot(int id);



private:
    Ui::rs_actioncompensate *ui;
};

#endif // RS_ACTIONCOMPENSATE_H
