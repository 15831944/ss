#include "sx_breakallgroup.h"

sx_BreakAllGroup::sx_BreakAllGroup(RS_EntityContainer& container,
        RS_GraphicView& graphicView)
        :RS_PreviewActionInterface("Blocks Create",container, graphicView)
        ,referencePoint(new RS_Vector{})
{
    container.setSelected(false);
    graphicView.saveView();
    sx_actionGroupExplode* blockexplode;
    actionType=RS2::ActionBlocksCreate;
    for(auto ec: container.getEntityList()){
        if(ec->rtti() == RS2::EntityInsert)
        {

            ec->setSelected(true);
            blockexplode  = new sx_actionGroupExplode(container, graphicView);
        }
    }
    graphicView.redraw(RS2::RedrawDrawing);
}

sx_BreakAllGroup::~sx_BreakAllGroup() = default;


void sx_BreakAllGroup::init(int status) {
    RS_ActionInterface::init(status);
    trigger();
    finish(false);
}

void sx_BreakAllGroup::trigger() {

    RS_DIALOGFACTORY->updateSelectionWidget(container->countSelected(),container->totalSelectedLength());
}
