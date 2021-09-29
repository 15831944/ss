#include "sx_actiongroupexplode.h"
#include <QAction>
#include "rs_modification.h"
#include "QDebug"

sx_actionGroupExplode::sx_actionGroupExplode(RS_EntityContainer& container,
        RS_GraphicView& graphicView)
        :RS_PreviewActionInterface("Blocks Explode",
                           container, graphicView) {
    actionType=RS2::ActionBlocksExplode;
    init();
}

void sx_actionGroupExplode::init(int status) {
    RS_PreviewActionInterface::init(status);
    trigger();
    finish(false);
}



void sx_actionGroupExplode::trigger() {
    RS_Modification m(*container, graphicView);
    m.explode();
    if(m.explodeNumber() == 0)
    {
        return;
    }
    tmpNumber = m.explodeNumber();
}

int sx_actionGroupExplode::size()
{
    return tmpNumber;
}
