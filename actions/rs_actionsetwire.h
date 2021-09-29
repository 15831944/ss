#ifndef RS_ACTIONSETWIRE_H
#define RS_ACTIONSETWIRE_H
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
#include "rs_actionsetwiredialog.h"

class rs_actionsetwire: public RS_ActionInterface
{
public:
    explicit rs_actionsetwire(RS_EntityContainer &container, RS_GraphicView &graphicView);
    rs_actionsetwiredialog*swd;
};

#endif // RS_ACTIONSETWIRE_H
