#ifndef RS_ACTIONREMOVESMALL_H
#define RS_ACTIONREMOVESMALL_H

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
#include "rs_actionremovesmalldialog.h"
class rs_actionremovesmall: public RS_ActionInterface
{
    Q_OBJECT
public:
    explicit rs_actionremovesmall(RS_EntityContainer &container, RS_GraphicView &graphicView);
    rs_actionremovesmalldialog*armd;
private slots:
    void myRemoveSmallSlot(double data,int flag);
    void receiveCloseSlot(bool isclosed);
};

#endif // RS_ACTIONREMOVESMALL_H
