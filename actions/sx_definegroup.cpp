#include "sx_definegroup.h"


sx_DefineGroup::sx_DefineGroup(RS_EntityContainer& container,
                               RS_GraphicView& graphicView)
    :RS_PreviewActionInterface("Blocks Create",container, graphicView)
    ,referencePoint(new RS_Vector{})
{
    actionType=RS2::ActionBlocksCreate;
//    for(auto ec:container.getEntityList())
//    {
//        if(ec->isSelected())
//        {
//            if (document) {
//               document->startUndoCycle();
//                document->addUndoable(ec);
//               document->endUndoCycle();
//            }
//        }
//    }
}

sx_DefineGroup::~sx_DefineGroup() = default;


void sx_DefineGroup::init(int status) {
    RS_PreviewActionInterface::init(status);
}



void sx_DefineGroup::trigger() {
    if (graphic) {
        RS_BlockList* blockList = graphic->getBlockList();

        if (blockList) {
            RS_BlockData d =
                    RS_DIALOGFACTORY->requestNewGroupDialog(blockList);
            if (!d.name.isEmpty()) {
                RS_Creation creation(container, graphicView);
                creation.createGroup(&d, *referencePoint, true);
                RS_InsertData id(
                            d.name,
                            *referencePoint,
                            RS_Vector(1.0,1.0),
                            0.0,
                            1, 1, RS_Vector(0.0,0.0)
                            );
                creation.createInsert(&id);
            }

        }
    }

    graphicView->redraw(RS2::RedrawDrawing);

    setStatus(getStatus()+1); // clear mouse button hints
    updateMouseButtonHints();
    graphicView->killSelectActions();
    finish(false);
}


void sx_DefineGroup::mouseMoveEvent(QMouseEvent* e) {
    snapPoint(e);

    switch (getStatus()) {
    case SetReferencePoint:

        break;

    default:
        break;
    }
}



void sx_DefineGroup::mouseReleaseEvent(QMouseEvent* e) {
    if (e->button()==Qt::LeftButton) {
        RS_CoordinateEvent ce(snapPoint(e));
        coordinateEvent(&ce);
    } else if (e->button()==Qt::RightButton) {
        init(getStatus()-1);
    }
}



void sx_DefineGroup::coordinateEvent(RS_CoordinateEvent* e) {
    if (!e) {
        return;
    }

    switch (getStatus()) {
    case SetReferencePoint:
        *referencePoint = e->getCoordinate();
        trigger();
        break;

    default:
        break;

    }
}



void sx_DefineGroup::updateMouseButtonHints() {
    switch (getStatus()) {
    case SetReferencePoint:
        RS_DIALOGFACTORY->updateMouseWidget(tr("Specify reference point"),
                                            tr("Cancel"));
        break;
    default:
        RS_DIALOGFACTORY->updateMouseWidget();
        break;
    }
}


void sx_DefineGroup::updateMouseCursor() {
    graphicView->setMouseCursor(RS2::CadCursor);
}
