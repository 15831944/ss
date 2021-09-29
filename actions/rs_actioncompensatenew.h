#ifndef RS_ACTIONCOMPENSATENEW_H
#define RS_ACTIONCOMPENSATENEW_H

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
#include "mrueditor.h"
#include "rs_actioncompensate.h"
class rs_actioncompensatenew : public RS_ActionInterface
{
    Q_OBJECT
public:
    explicit rs_actioncompensatenew(RS_EntityContainer &container, RS_GraphicView &graphicView,QWidget *parent = nullptr);
    rs_actioncompensate*cps;
};

#endif // RS_ACTIONCOMPENSATENEW_H
