#ifndef SX_RECTANGLEARRAY_H
#define SX_RECTANGLEARRAY_H


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
#include "QVBoxLayout"
#include "QGroupBox"
#include "QLineEdit"
#include "QRadioButton"
#include "sx_rectarray.h"
#include "rs_modification.h"
#include "lc_undosection.h"
#include "rs_actionblocksexplode.h"

class sx_rectangleArray : public RS_PreviewActionInterface
{
    Q_OBJECT
public:
    sx_rectangleArray(RS_EntityContainer &container, RS_GraphicView &graphicVi
                       ,RS_ActionInterface* actionSelect=nullptr,
                       std::initializer_list<RS2::EntityType> const& entityTypeList=std::initializer_list<RS2::EntityType>{});
    ~sx_rectangleArray();
    void init(int status=0) override;

    void trigger() override;

private:
    std::initializer_list<RS2::EntityType> const entityTypeList;
    RS_Entity* en;
    RS_ActionInterface* actionSelect;
    void addNewEntities(std::vector<RS_Entity*>& addList);
//    QDialog *dialog;
    sx_rectArray *array;

private slots:
    void arrayDataSlot(int,int,bool,bool,double,double,bool,bool,bool,bool,double,double);
    void receiveCloseSlot(bool);
};

#endif // SX_RECTANGLEARRAY_H
