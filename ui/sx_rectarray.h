#ifndef SX_RECTARRAY_H
#define SX_RECTARRAY_H

#include <QDialog>
#include <QAction>
#include <QMouseEvent>
#include "rs_actionblockscreate.h"

#include "rs_creation.h"
#include "rs_dialogfactory.h"
#include "rs_graphicview.h"
#include "rs_graphic.h"
#include "rs_insert.h"
#include "rs_modification.h"
#include "rs_coordinateevent.h"
#include "QDebug"
#include "QToolBar"
#include "QLabel"
#include "QObject"
#include "QMessageBox"
#include "QIntValidator"
//#include "sx_rectanglearray.h"
#include "QLineEdit"

namespace Ui {
class sx_rectArray;
}

class sx_rectArray : public QDialog
{
    Q_OBJECT
public:
    explicit sx_rectArray(RS_EntityContainer &container,
                          RS_GraphicView &graphicVi,
                          QWidget *parent = nullptr);
    ~sx_rectArray();

private:
    Ui::sx_rectArray *ui;
    std::initializer_list<RS2::EntityType> const entityTypeList;
    RS_Entity* en;
    RS_ActionInterface* actionSelect;

private slots:
    void slotArrayNumber();
    void slotSpacing();
    void slotDeviation();
    void slotCloseWindow();

protected:
    virtual void closeEvent(QCloseEvent * event);

signals:
    void rectArrayDataSignal(int,int,bool,bool,double,double,bool,bool,bool,bool,double,double);
    void closeWindowSignal(bool isClose = false);
};

#endif // SX_RECTARRAY_H
