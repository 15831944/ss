#include "sx_smallsizeselect.h"

sx_SmallSizeSelect::sx_SmallSizeSelect(RS_EntityContainer &container, RS_GraphicView &graphicView
                                       ,RS_ActionInterface* action_select,
                                       std::initializer_list<RS2::EntityType> const& entityTypeList)
    :RS_PreviewActionInterface("SmallSize Selected", container, graphicView)
    ,entityTypeList(entityTypeList)
    ,en(nullptr)
    ,actionSelect(action_select)
{

}

void sx_SmallSizeSelect::cmdChanged()
{
    for(auto ec: container->getEntityList()){
        if((ec->getSize().x >= ec->getSize().y?ec->getSize().x:ec->getSize().y) < cmd.toDouble()){
            qDebug() << "cmd" << cmd;
            ec->setSelected(true);
        }
        else
             ec->setSelected(false);
    }

    graphicView->redraw(RS2::RedrawDrawing);
}

void sx_SmallSizeSelect::commandEvent(RS_CommandEvent* e) {
    cmd = e->getCommand().toLower();
    if(cmd.toDouble() <= 0 || cmd.toDouble() >= 99999)
    {
        RS_DIALOGFACTORY->commandMessage("Invalid input");
    }
    if (checkCommand("help", cmd)) {
        RS_DIALOGFACTORY->commandMessage(msgAvailableCommands()
                                         + getAvailableCommands().join(", "));
        return;
    }
    e->accept();
    cmdChanged();
}



QStringList sx_SmallSizeSelect::getAvailableCommands() {
    QStringList cmd;
    return cmd;
}


void sx_SmallSizeSelect::updateMouseButtonHints() {   //命令行上的字
    switch (getStatus()) {
    case 0:
        RS_DIALOGFACTORY->updateMouseWidget(
                    tr("Please specify the size"), tr("Back"));
        break;
//    case 1:
//        RS_DIALOGFACTORY->updateMouseWidget(
//                   tr("Back"));
//        break;
    default:
        RS_DIALOGFACTORY->updateMouseWidget();
        break;
    }
}

void sx_SmallSizeSelect::trigger() {
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


void sx_SmallSizeSelect::keyPressEvent(QKeyEvent* e)
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


void sx_SmallSizeSelect::mouseReleaseEvent(QMouseEvent* e)
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


void sx_SmallSizeSelect::updateMouseCursor() {
    graphicView->setMouseCursor(RS2::CrossCursor);
}
