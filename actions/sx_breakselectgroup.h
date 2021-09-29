#ifndef SX_BREAKSELECTGROUP_H
#define SX_BREAKSELECTGROUP_H

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

//RS_ActionBlocksExplode
class sx_BreakSelectGroup : public RS_PreviewActionInterface
{
public:
    sx_BreakSelectGroup();

    sx_BreakSelectGroup(RS_EntityContainer& container,
                        RS_GraphicView& graphicView);
    ~sx_BreakSelectGroup() override;

    void init(int status=0) override;

    void trigger() override;

protected:
    std::unique_ptr<RS_Vector> referencePoint;
};

#endif // SX_BREAKSELECTGROUP_H
