#ifndef SX_BREAKALLGROUP_H
#define SX_BREAKALLGROUP_H

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
#include "rs_creation.h"
#include "qc_applicationwindow.h"
#include "qg_actionhandler.h"
#include "rs_actionblocksexplode.h"
#include "rs_actionselect.h"
#include "rs_selection.h"
#include "sx_actiongroupexplode.h"
#include "rs_modification.h"

class sx_BreakAllGroup : public RS_PreviewActionInterface
{
public:
    sx_BreakAllGroup();

    sx_BreakAllGroup(RS_EntityContainer& container,
                        RS_GraphicView& graphicView);
    ~sx_BreakAllGroup() override;

    void init(int status=0) override;

    void trigger() override;

protected:
    std::unique_ptr<RS_Vector> referencePoint;
};

#endif // SX_BREAKALLGROUP_H
