#ifndef RS_ACTIONMERGENEW_H
#define RS_ACTIONMERGENEW_H
#include <QObject>
#include <QDialog>
#include"rs_entitycontainer.h"
#include <QMouseEvent>
#include"rs_entity.h"
#include"rs_modification.h"
#include "rs_vector.h"
#include "QPoint"
#include"rs_graphicview.h"
#include "rs_previewactioninterface.h"
#include"rs_actionmerge.h"
#include"rs_line.h"
#include"rs_block.h"
#include"rs_blocklist.h"
#include"rs_graphic.h"
#include"rs_creation.h"
#include"rs_insert.h"
#include"rs_dialogfactory.h"
class rs_actionmergenew: public RS_ActionInterface
{
    Q_OBJECT
public:
    explicit rs_actionmergenew(RS_EntityContainer &container, RS_GraphicView &graphicView);
    rs_actionmerge*am;
    RS_Line*line;

private slots:
    void myMergeSignal(double data);
    void receiveCloseSlot(bool isClose);

};

#endif // RS_ACTIONMERGENEW_H
