#include "rs_actionremovesmall.h"
#include<QDebug>
rs_actionremovesmall::rs_actionremovesmall(RS_EntityContainer &container, RS_GraphicView &graphicView) :
    RS_ActionInterface("WeiLian Entities", container, graphicView){
    armd = new rs_actionremovesmalldialog();
    connect(armd,SIGNAL(myRemoveSmallSignal(double,int)),this,SLOT(myRemoveSmallSlot(double,int)));
    connect(armd,SIGNAL(closeWindowSignal(bool)),this,SLOT(receiveCloseSlot(bool)));
    armd->exec();
}

void rs_actionremovesmall::myRemoveSmallSlot(double data,int flag)
{
    if(!flag){
        finish();
        return;
    }
    double maxlength = 0;
    std::vector<RS_Entity*> addList;
    for(auto ec: container->getEntityList()){
        maxlength = ec->getSize().x<ec->getSize().y?ec->getSize().y:ec->getSize().x;
        if(maxlength<=data){
            addList.push_back(ec);
        }

    }
    for (RS_Entity* e: addList) {
        container->removeEntity(e);
    }

    graphicView->redraw();
    finish();
}

void rs_actionremovesmall::receiveCloseSlot(bool isClosed)
{
    if(isClosed)
        {
            finish(false);
            armd->close();
        }
}
