#ifndef SX_ARRAYDATA_H
#define SX_ARRAYDATA_H

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
#include "sx_interactivearray.h"
#include "rs_actionblocksexplode.h"
#include "rs_polyline.h"
#include "rs_point.h"
#include "rs_preview.h"
#include "rs_commandevent.h"
#include "rs_line.h"



class sx_arrayData : public RS_PreviewActionInterface
{
    Q_OBJECT
public:
    enum Status {
        SetCorner1,      /**< Setting the 1st corner.  */
        SetCorner2       /**< Setting the 2nd corner. */
    };
    sx_arrayData(RS_EntityContainer &container, RS_GraphicView &graphicVi
                       ,RS_ActionInterface* actionSelect=nullptr,
                       std::initializer_list<RS2::EntityType> const& entityTypeList=std::initializer_list<RS2::EntityType>{});
    void mouseMoveEvent(QMouseEvent* e) override;
    void mouseReleaseEvent(QMouseEvent* e) override;

    void coordinateEvent(RS_CoordinateEvent* e) override;
    void updateMouseCursor() override;
private:
    std::initializer_list<RS2::EntityType> const entityTypeList;
    RS_Entity* en;
    RS_ActionInterface* actionSelect;
    sx_interactiveArray *interArray;
//    void init(int status=0) override;
    bool openEvent = false;
    std::vector<RS_Entity*> addList;
  //  std::vector<RS_Entity*>& tmpList;
    void trigger() override;
    void addNewEntities(std::vector<RS_Entity*>& addList);
    int i = 1;
    double onePointX = 0,onePointY = 0;
    double tmpPointX = 0,tmpPointY = 0;
    QList<double> list;
    RS_Entity *entity;
    RS_Vector *vector_row;
    RS_Vector *vector_column;
    RS_Vector *vector_row2;
    RS_Vector *vector_column2;
    double spacingRow = 0,spacingColumn = 0;
    bool isDeleteEntity = false;

private slots:
    void receiveDataSlot(bool,bool,double,double);
    void receiveCloseSlot(bool);

protected:
    struct Points;
    struct Points *pPoints;
//    std::unique_ptr<Points> pPoints;
};

#endif // SX_ARRAYDATA_H
