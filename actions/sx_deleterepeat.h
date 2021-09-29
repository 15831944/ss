#ifndef DELETEREPEAT_H
#define DELETEREPEAT_H

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
#include "rs_line.h"
#include "rs_linetypepattern.h"
#include "rs_actiondrawline.h"



class deleteRepeat: public RS_PreviewActionInterface
{
public:
    deleteRepeat(RS_EntityContainer& container,
                        RS_GraphicView& graphicView);
    std::vector<RS_Entity*> list;
    std::vector<RS_Entity*> removeList;
        int flag = 0;
protected:
    std::unique_ptr<RS_Vector> referencePoint;

private:
    double xDifference;
    double yDifference;
    QList<RS_Vector*> pointList;
    QList<RS_Entity*> entityList;
    RS_Entity *ec;
    RS_Entity *ec2;
    int count;
    int number;
   // RS_Line *line;
    RS_Vector *point1;
    RS_Vector *point2;
};

#endif // DELETEREPEAT_H
