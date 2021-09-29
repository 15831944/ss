#include "rs_actionalignleft.h"
#include<QDebug>
#include"rs_modification.h"
#include "lc_undosection.h"
RS_ActionAlign::RS_ActionAlign(RS_EntityContainer &container, RS_GraphicView &graphicView,int id)
    :RS_PreviewActionInterface("Align Left Entities", container, graphicView)
{

   // qDebug()<<id;
    switch (id){
        case 1:alignLeft();break;
        case 2:alignRight();break;
        case 3:alignHorizontal();break;
        case 4:alignTop();break;
        case 5:alignBottom();break;
        case 6:verticalCenter();break;
        case 7:/*alignHorizontal();
               verticalCenter();*/
                centerAlign();
               break;
        default:break;
    }
    LC_UndoSection undo(container.getDocument());
    graphicView.redraw(RS2::RedrawDrawing); 
    finish();
}
RS_ActionAlign::~RS_ActionAlign() = default;

void RS_ActionAlign::mouseReleaseEvent(QMouseEvent* e) {
    if (e->button()==Qt::RightButton) {
        init(getStatus()-1);
    }
}

void RS_ActionAlign::init(int status) {
    RS_ActionInterface::init(status);
}

void RS_ActionAlign::alignLeft()
{
    std::vector<RS_Entity*> addList;
    std::vector<RS_Entity*> removeList;
    double minx;
    for(auto ec: container->getEntityList()){
        if (ec->isSelected()) {
            minx = ec->getMin().x;
            break;
        }
    }
    for(auto ec: container->getEntityList()){
        if (ec->isSelected()) {
            if(minx>ec->getMin().x)
                minx = ec->getMin().x;
        }
    }

    RS_Vector v1(minx,0);
    for(auto ec: container->getEntityList()){
        if (ec->isSelected()) {
            v1.x = minx-ec->getMin().x;
            RS_Entity*e = ec->clone();
            e->move(v1);
            removeList.push_back(ec);
            addList.push_back(e);
        }
    }
    LC_UndoSection undo1( document);
    for(auto ec: addList){
        container->addEntity(ec);
        undo1.addUndoable(ec);
    }
    for(auto ec: removeList){
        //ec->setVisible(0);
       /* graphicView->deleteEntity(ec);
        */ec->changeUndoState();
        undo1.addUndoable(ec);
    }
}

void RS_ActionAlign::alignRight()
{
    std::vector<RS_Entity*> addList;
    std::vector<RS_Entity*> removeList;
    double maxx;
    for(auto ec: container->getEntityList()){
        if (ec->isSelected()) {
            maxx = ec->getMax().x;
            break;
        }
    }
    for(auto ec: container->getEntityList()){
        if (ec->isSelected()) {
            if(maxx<ec->getMax().x)
                maxx = ec->getMax().x;
        }
    }
    RS_Vector v1(maxx,0);
    for(auto ec: container->getEntityList()){
        if (ec->isSelected()) {
            v1.x = maxx-ec->getMax().x;
            RS_Entity*e = ec->clone();
            e->move(v1);
            removeList.push_back(ec);
            addList.push_back(e);
        }
    }
    LC_UndoSection undo1( document);
    for(auto ec: addList){
        container->addEntity(ec);
        undo1.addUndoable(ec);
    }
    for(auto ec: removeList){
        //ec->setVisible(0);
        graphicView->deleteEntity(ec);
        ec->changeUndoState();
        undo1.addUndoable(ec);
    }
//    LC_UndoSection undo( document, undo1);
}

void RS_ActionAlign::alignHorizontal()
{
    std::vector<RS_Entity*> addList;
    std::vector<RS_Entity*> removeList;
    double sum = 0;
    int count = 0;
    RS_Vector v1(0,0);
    for(auto ec: container->getEntityList()){
        if (ec->isSelected()) {
            sum += (ec->getMax().x + ec->getMin().x)/2;
            count++;
        }
    }
    double averagecenter = sum/count;
    for(auto ec: container->getEntityList()){
        if (ec->isSelected()) {
            v1.x = averagecenter - (ec->getMax().x + ec->getMin().x)/2;
            RS_Entity*e = ec->clone();
            e->move(v1);
            removeList.push_back(ec);
            addList.push_back(e);
        }
    }
    LC_UndoSection undo1( document);
    for(auto ec: addList){
        container->addEntity(ec);
        undo1.addUndoable(ec);
    }
    for(auto ec: removeList){
        //ec->setVisible(0);
        graphicView->deleteEntity(ec);
        ec->changeUndoState();
        undo1.addUndoable(ec);
    }
}

void RS_ActionAlign::alignTop()
{
    std::vector<RS_Entity*> addList;
    std::vector<RS_Entity*> removeList;
    double maxy;
    for(auto ec: container->getEntityList()){
        if (ec->isSelected()) {
            maxy = ec->getMax().y;
            break;
        }
    }
    for(auto ec: container->getEntityList()){
        if (ec->isSelected()) {
            if(maxy<ec->getMax().y)
                maxy = ec->getMax().y;
        }
    }
    RS_Vector v1(0,maxy);
    for(auto ec: container->getEntityList()){
        if (ec->isSelected()) {
            v1.y = maxy-ec->getMax().y;
            RS_Entity*e = ec->clone();
            e->move(v1);
            removeList.push_back(ec);
            addList.push_back(e);
        }
    }
    LC_UndoSection undo1( document);
    for(auto ec: addList){
        container->addEntity(ec);
        undo1.addUndoable(ec);
    }
    for(auto ec: removeList){
        //ec->setVisible(0);
        graphicView->deleteEntity(ec);
        ec->changeUndoState();
        undo1.addUndoable(ec);
    }
}

void RS_ActionAlign::alignBottom()
{
    std::vector<RS_Entity*> addList;
    std::vector<RS_Entity*> removeList;
    double miny;
    for(auto ec: container->getEntityList()){
        if (ec->isSelected()) {
            miny = ec->getMin().y;
            break;
        }
    }
    for(auto ec: container->getEntityList()){
        if (ec->isSelected()) {
            if(miny>ec->getMin().y)
                miny = ec->getMin().y;
        }
    }
    RS_Vector v1(0,miny);
    for(auto ec: container->getEntityList()){
        if (ec->isSelected()) {
            v1.y = miny-ec->getMin().y;
            RS_Entity*e = ec->clone();
            e->move(v1);
            removeList.push_back(ec);
            addList.push_back(e);
        }
    }
    LC_UndoSection undo1( document);
    for(auto ec: addList){
        container->addEntity(ec);
        undo1.addUndoable(ec);
    }
    for(auto ec: removeList){
        //ec->setVisible(0);
        graphicView->deleteEntity(ec);
        ec->changeUndoState();
        undo1.addUndoable(ec);
    }

}

void RS_ActionAlign::verticalCenter()
{
    std::vector<RS_Entity*> addList;
    std::vector<RS_Entity*> removeList;
    double sum = 0;
    int count = 0;
    RS_Vector v1(0,0);
    for(auto ec: container->getEntityList()){
        if (ec->isSelected()) {
            sum += (ec->getMax().y + ec->getMin().y)/2;
            count++;
        }
    }
    double averagecenter = sum/count;
    for(auto ec: container->getEntityList()){
        if (ec->isSelected()) {
            v1.y = averagecenter - (ec->getMax().y + ec->getMin().y)/2;
            RS_Entity*e = ec->clone();
            e->move(v1);
            removeList.push_back(ec);
            addList.push_back(e);
        }
    }
    LC_UndoSection undo1( document);
    for(auto ec: addList){
        container->addEntity(ec);
        undo1.addUndoable(ec);
    }
    for(auto ec: removeList){
        //ec->setVisible(0);
        graphicView->deleteEntity(ec);
        ec->changeUndoState();
        undo1.addUndoable(ec);
    }
}

void RS_ActionAlign::centerAlign()
{
    std::vector<RS_Entity*> addList;
    std::vector<RS_Entity*> removeList;
    double sum = 0;
    int count = 0;
    RS_Vector v1(0,0);
    double sum1 = 0;
    for(auto ec: container->getEntityList()){
        if (ec->isSelected()) {
            sum += (ec->getMax().x + ec->getMin().x)/2;
            count++;
            sum1 += (ec->getMax().y + ec->getMin().y)/2;
        }
    }
    double averagecenter = sum/count;
    double averagecenter1 = sum1/count;
    for(auto ec: container->getEntityList()){
        if (ec->isSelected()) {
            v1.x = averagecenter - (ec->getMax().x + ec->getMin().x)/2;
            v1.y = averagecenter1 - (ec->getMax().y + ec->getMin().y)/2;
            RS_Entity*e = ec->clone();
            e->move(v1);
            removeList.push_back(ec);
            addList.push_back(e);
        }
    }

    LC_UndoSection undo1( document);
    for(auto ec: addList){
        container->addEntity(ec);
        undo1.addUndoable(ec);
    }
    for(auto ec: removeList){
        //ec->setVisible(0);
        graphicView->deleteEntity(ec);
        ec->changeUndoState();
        undo1.addUndoable(ec);
    }
}

