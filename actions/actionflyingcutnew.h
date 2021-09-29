#ifndef ACTIONFLYINGCUTNEW_H
#define ACTIONFLYINGCUTNEW_H
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
#include "rs_actionflyingcut.h"
#include "rs_actionflyingcutarc.h"


class actionflyingcutnew: public RS_ActionInterface
{
    Q_OBJECT
public:
    explicit actionflyingcutnew(RS_EntityContainer &container, RS_GraphicView &graphicView,int id);
    rs_actionflyingcut*fc;
    rs_actionflyingcutarc*fca;
};

#endif // ACTIONFLYINGCUTNEW_H
