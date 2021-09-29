#include "sx_breakselectgroup.h"
#include "rs_selection.h"

sx_BreakSelectGroup::sx_BreakSelectGroup(RS_EntityContainer& container,
        RS_GraphicView& graphicView)
        :RS_PreviewActionInterface("Blocks Create",container, graphicView)
        ,referencePoint(new RS_Vector{})
{
    graphicView.saveView();
    actionType=RS2::ActionBlocksCreate;
    for(auto ec: container.getEntityList()){
        if(ec->isSelected())
        {
            if(ec->rtti() == RS2::EntityInsert)
            {
               RS_ActionBlocksExplode* blockexplode  = new RS_ActionBlocksExplode(container, graphicView);

            }
        }
    }
    graphicView.redraw(RS2::RedrawDrawing);
}

sx_BreakSelectGroup::~sx_BreakSelectGroup() = default;




void sx_BreakSelectGroup::init(int status) {
    RS_ActionInterface::init(status);
    trigger();
    finish(false);
}

void sx_BreakSelectGroup::trigger() {
//    RS_Selection s(*container, graphicView);
//    s.selectAll(true);

    RS_DIALOGFACTORY->updateSelectionWidget(container->countSelected(),container->totalSelectedLength());
}

