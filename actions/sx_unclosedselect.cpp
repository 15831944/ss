#include "sx_unclosedselect.h"
#include<QDebug>
#include "rs_debug.h"
#include"rs_polyline.h"
#include"rs_entity.h"
#include"rs_entitycontainer.h"
sx_UnclosedSelect::sx_UnclosedSelect(RS_EntityContainer &container, RS_GraphicView &graphicView
                                     ,RS_ActionInterface* action_select,
                                     std::initializer_list<RS2::EntityType> const& entityTypeList)

 :RS_PreviewActionInterface("Unclosed Selected", container, graphicView)
 ,entityTypeList(entityTypeList)
 ,en(nullptr)
 ,actionSelect(action_select)
{
    RS_Polyline *polyine;
    for(auto ec: container.getEntityList()){
       switch(ec->rtti())
        {
            case RS2::EntityPoint:
                ec->setSelected(true);
                break;
            case RS2::EntityLine:
                ec->setSelected(true);
                break;
           case RS2::EntityPolyline:
                polyine = (RS_Polyline*)ec;
                ec->setSelected(!polyine->isClosed());
                break;
            case RS2::EntityArc:
                ec->setSelected(true);
                break;
            case RS2::EntityHyperbola:
                ec->setSelected(true);
                break;
            case RS2::EntityConstructionLine:
                ec->setSelected(true);
                break;
             case RS2::EntitySpline:
                ec->setSelected(true);
                break;
             case RS2::EntitySplinePoints:
                ec->setSelected(true);
                break;
             default:
                ec->setSelected(false);
                break;
        }
    }
    graphicView.redraw(RS2::RedrawDrawing);
}

void sx_UnclosedSelect::trigger() {
    bool typeMatch{true};
    if (en && entityTypeList.size())
        typeMatch = std::find(entityTypeList.begin(), entityTypeList.end(), en->rtti())
                != entityTypeList.end();
    if (en && typeMatch) {
        RS_Selection s(*container, graphicView);
        s.selectSingle(en);

        RS_DIALOGFACTORY->updateSelectionWidget(container->countSelected(),container->totalSelectedLength());
    }
    else {
        RS_DEBUG->print("RS_ActionSelectSingle::trigger: Entity is NULL\n");
    }
}


void sx_UnclosedSelect::keyPressEvent(QKeyEvent* e)
{
    if (e->key()==Qt::Key_Escape)
    {
        finish(false);
        actionSelect->keyPressEvent(e);
    }

    if (container->countSelected() > 0 && e->key()==Qt::Key_Enter)
    {
        finish(false);
        actionSelect->keyPressEvent(e);
    }
}


void sx_UnclosedSelect::mouseReleaseEvent(QMouseEvent* e)
{
    if (e->button()==Qt::RightButton)
    {
        finish();
        if (actionSelect->rtti() == RS2::ActionSelect)
            actionSelect->finish();
        else
            actionSelect->mouseReleaseEvent(e);
    }
    else
    {
        if (entityTypeList.size()) {
            en = catchEntity(e, entityTypeList);
        }else{
            en = catchEntity(e);
        }
        trigger();
    }
}

void sx_UnclosedSelect::updateMouseCursor() {
    graphicView->setMouseCursor(RS2::ArrowCursor);
}
