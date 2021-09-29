#include "sx_arraydata.h"


struct sx_arrayData::Points {
    /**
     * 1st corner.
     */
    RS_Vector corner1;
    /**
     * 2nd corner.
     */
    RS_Vector corner2;
};

sx_arrayData::sx_arrayData(RS_EntityContainer &container, RS_GraphicView &graphicView
                                       ,RS_ActionInterface* action_select,
                                       std::initializer_list<RS2::EntityType> const& entityTypeList)
    :RS_PreviewActionInterface("SmallSize Selected", container, graphicView)
    ,entityTypeList(entityTypeList)
    ,en(nullptr)
    ,actionSelect(action_select)
    ,pPoints(new Points{})
{
    interArray = new sx_interactiveArray();
    interArray->setWindowModality(Qt::ApplicationModal);
    interArray->show();

    connect(interArray,SIGNAL(sendDataSignal(bool,bool,double,double)),this,SLOT(receiveDataSlot(bool,bool,double,double)));
    connect(interArray,SIGNAL(closeWindowSignal(bool)),this,SLOT(receiveCloseSlot(bool)));
}


void sx_arrayData::trigger() {

    RS_PreviewActionInterface::trigger();

    RS_Polyline* polyline = new RS_Polyline(container);

    // create and add rectangle:
    polyline->addVertex(pPoints->corner1);
    polyline->setLayerToActive();
    polyline->setPenToActive();
    polyline->addVertex({pPoints->corner2.x, pPoints->corner1.y});
    polyline->addVertex(pPoints->corner2);
    polyline->addVertex({pPoints->corner1.x, pPoints->corner2.y});
    polyline->setClosed(true);
    polyline->endPolyline();
  //  container->addEntity(polyline);

    // upd. undo list:
    if (document) {
        document->startUndoCycle();
        document->addUndoable(polyline);
        document->endUndoCycle();
    }

    // upd. view
    graphicView->redraw(RS2::RedrawDrawing);
    graphicView->moveRelativeZero(pPoints->corner2);

    RS_DIALOGFACTORY->updateSelectionWidget(container->countSelected(),container->totalSelectedLength());

    for(RS_Entity* ec: addList){  //打散群组
        if(ec->rtti() == RS2::EntityInsert)
        {
            ec->setSelected(1);
        }
    }
    entity->setSelected(true);
    if(isDeleteEntity == true)
    {
        container->removeEntity(entity);
        graphicView->redraw(RS2::RedrawDrawing);
    }
    RS_ActionBlocksExplode* blockexplode  = new RS_ActionBlocksExplode(*container, *graphicView);
    container->setSelected(false);

    finish(false);
}



void sx_arrayData::addNewEntities(std::vector<RS_Entity*>& addList)
{

    //    LC_UndoSection undo( document, handleUndo);
    for (RS_Entity* e: addList) {
        if (e) {
            container->addEntity(e);
            //            undo.addUndoable(e);
        }
    }

    container->calculateBorders();

    if (graphicView) {
        graphicView->redraw(RS2::RedrawDrawing);
    }
}



void sx_arrayData::mouseMoveEvent(QMouseEvent* e) {
    if(openEvent == true)
    {
        RS_Vector mouse = snapPoint(e);
        if (getStatus()==SetCorner2 && pPoints->corner1.valid) {   //当左上角被按下鼠标在移动就是下方代码
            pPoints->corner2 = mouse;
            deletePreview();
            preview->addRectangle(pPoints->corner1, pPoints->corner2);
            drawPreview();

            auto ec = entity;
            if((preview.get()->getSize().x > ec->getSize().x+spacingRow)&&(preview.get()->getSize().y > ec->getSize().y+spacingColumn))  //判断矩形框是否大于所要阵列的图形
            {
                if(((preview.get()->getSize().x / (ec->getSize().x+spacingRow)) * (preview.get()->getSize().y / (ec->getSize().y+spacingColumn))) > addList.size())  //判断实际阵列数量是否大于计算数量
                {
                    if(addList.size() > 1000)
                    {
                        return;
                    }
                    for (RS_Entity* e: addList) {
                        container->removeEntity(e);
                    }
                    addList.clear();

                    for(int t = 0;t < preview.get()->getSize().x / (ec->getSize().x+spacingRow) - 1;t++)
                    {
                        RS_Entity* et = ec->clone();
                        if((pPoints->corner2.x >= pPoints->corner1.x)&&(pPoints->corner2.y < pPoints->corner1.y))
                        {
                            //左上往右下
                            vector_row = new RS_Vector(onePointX-et->getMin().x + t * ec->getSize().x + t * spacingRow,onePointY-et->getMin().y-et->getSize().y);  //当矩形框是从左到右时，将阵列源图移动到矩形框内
                        }
                        else if((pPoints->corner2.x < pPoints->corner1.x)&&(pPoints->corner2.y < pPoints->corner1.y))
                        {
                            //右上往左下
                            vector_row = new RS_Vector(onePointX-et->getMax().x - t * ec->getSize().x - t * spacingRow,onePointY-et->getMin().y-et->getSize().y);
                        }
                        else if((pPoints->corner2.x > pPoints->corner1.x)&&(pPoints->corner2.y >= pPoints->corner1.y))
                        {

                            vector_row = new RS_Vector(onePointX-et->getMin().x + t * (ec->getSize().x + spacingRow),onePointY-et->getMin().y);
                        }
                        else if((pPoints->corner2.x < pPoints->corner1.x)&&(pPoints->corner2.y > pPoints->corner1.y))
                        {
                            vector_row = new RS_Vector(onePointX-et->getMax().x - t * (ec->getSize().x + spacingRow),onePointY-et->getMin().y);
                        }
                        et->move(*vector_row);
                        addList.push_back(et);

                        for(int p = 1;p < preview.get()->getSize().y / (ec->getSize().y+spacingColumn) - 1;p++)
                        {
                            RS_Entity* et = ec->clone();
                            if((pPoints->corner2.x >= pPoints->corner1.x)&&(pPoints->corner2.y < pPoints->corner1.y))
                            {
                                vector_column = new RS_Vector(onePointX-et->getMin().x + t * ec->getSize().x + t * spacingRow,(onePointY - et->getMin().y) - (et->getSize().y*(p+1)) - p * spacingColumn);  //当矩形框是从左到右时，将阵列源图移动到矩形框内
                            }
                            else if((pPoints->corner2.x < pPoints->corner1.x)&&(pPoints->corner2.y < pPoints->corner1.y))
                            {
                                vector_column = new RS_Vector(onePointX-et->getMax().x - t * ec->getSize().x - t * spacingRow,(onePointY - et->getMin().y) - (et->getSize().y*(p+1)) - p * spacingColumn);
                            }
                            else if((pPoints->corner2.x > pPoints->corner1.x)&&(pPoints->corner2.y >= pPoints->corner1.y))
                            {
                                vector_column = new RS_Vector(onePointX-et->getMin().x + t * (ec->getSize().x + spacingRow),(onePointY - et->getMin().y) + (et->getSize().y * p) + p * spacingColumn);
                            }
                            else if((pPoints->corner2.x < pPoints->corner1.x)&&(pPoints->corner2.y > pPoints->corner1.y))
                            {
                                vector_column = new RS_Vector(onePointX-et->getMax().x - t * (ec->getSize().x + spacingRow),(onePointY - et->getMin().y) + (et->getSize().y*p) + p * spacingColumn);
                            }
                            et->move(*vector_column);
                            addList.push_back(et);
                        }
                    }
                    addNewEntities(addList);
                }else if(((preview.get()->getSize().x / (ec->getSize().x+spacingRow)) * (preview.get()->getSize().y / (ec->getSize().y+spacingColumn))) < addList.size()){

                    for (RS_Entity* e: addList) {
                        container->removeEntity(e);
                    }
                    addList.clear();

                    for(int t = 0;t < preview.get()->getSize().x / (ec->getSize().x+spacingRow) - 1;t++)
                    {
                        RS_Entity* et = ec->clone();
                        if((pPoints->corner2.x >= pPoints->corner1.x)&&(pPoints->corner2.y < pPoints->corner1.y))
                        {
                            //左上往右下
                            vector_row2 = new RS_Vector(onePointX-et->getMin().x + t * (ec->getSize().x + spacingRow),onePointY - et->getMin().y - et->getSize().y);
                        }
                        else if((pPoints->corner2.x < pPoints->corner1.x)&&(pPoints->corner2.y < pPoints->corner1.y))
                        {
                            //右上往左下
                            vector_row2 = new RS_Vector(onePointX-et->getMax().x - t * (ec->getSize().x + spacingRow),onePointY - et->getMin().y - et->getSize().y);
                        }
                        else if((pPoints->corner2.x > pPoints->corner1.x)&&(pPoints->corner2.y >= pPoints->corner1.y))
                        {

                            vector_row2 = new RS_Vector(onePointX-et->getMin().x + t * (ec->getSize().x + spacingRow),onePointY-et->getMin().y);
                        }
                        else if((pPoints->corner2.x < pPoints->corner1.x)&&(pPoints->corner2.y > pPoints->corner1.y))
                        {
                            vector_row2 = new RS_Vector(onePointX-et->getMax().x - t * (ec->getSize().x + spacingRow),onePointY-et->getMin().y);
                        }
                        et->move(*vector_row2);

                        addList.push_back(et);

                        for(int p = 1;p < preview.get()->getSize().y / (ec->getSize().y+spacingColumn) - 1;p++)
                        {
                            RS_Entity* et = ec->clone();
                            if((pPoints->corner2.x >= pPoints->corner1.x)&&(pPoints->corner2.y < pPoints->corner1.y))
                            {
                                //左上往右下
                                vector_column2 = new RS_Vector(onePointX-et->getMin().x + t * (ec->getSize().x + spacingRow),onePointY-et->getMin().y - p*et->getSize().y - (p-1) * spacingColumn);
                            }
                            else if((pPoints->corner2.x < pPoints->corner1.x)&&(pPoints->corner2.y < pPoints->corner1.y))
                            {
                                //右上往左下
                                vector_column2 = new RS_Vector(onePointX-et->getMax().x - t * (ec->getSize().x + spacingRow),onePointY-et->getMin().y - p*et->getSize().y - (p-1) * spacingColumn);
                            }
                            else if((pPoints->corner2.x > pPoints->corner1.x)&&(pPoints->corner2.y >= pPoints->corner1.y))
                            {

                                vector_column2 = new RS_Vector(onePointX-et->getMin().x + t * (ec->getSize().x + spacingRow),onePointY-et->getMin().y + p * et->getSize().y + p * spacingColumn);
                            }
                            else if((pPoints->corner2.x < pPoints->corner1.x)&&(pPoints->corner2.y > pPoints->corner1.y))
                            {
                                vector_column2 = new RS_Vector(onePointX-et->getMax().x - t * (ec->getSize().x + spacingRow),onePointY-et->getMin().y + p * et->getSize().y + p * spacingColumn);
                            }
                            et->move(*vector_column2);
                            addList.push_back(et);
                        }
                    }
                    addNewEntities(addList);
                }
            }
            else if((preview.get()->getSize().x > (ec->getSize().x+spacingRow))&&(preview.get()->getSize().y < (ec->getSize().y+spacingColumn)))
            {

                for (RS_Entity* e: addList) {
                    container->removeEntity(e);
                }
                addList.clear();
                graphicView->redraw(RS2::RedrawDrawing);
            }
            else if((preview.get()->getSize().x < (ec->getSize().x+spacingRow))&&(preview.get()->getSize().y > (ec->getSize().y+spacingColumn)))
            {
                for (RS_Entity* e: addList) {
                    container->removeEntity(e);
                }
                addList.clear();
                graphicView->redraw(RS2::RedrawDrawing);
            }
            else if(addList.size() == 1)
            {
                for (RS_Entity* e: addList) {
                    container->removeEntity(e);
                }
                addList.clear();
                graphicView->redraw(RS2::RedrawDrawing);
            }
        }
    }
}

void sx_arrayData::mouseReleaseEvent(QMouseEvent* e) {
    if(openEvent == true)
    {
        if (e->button()==Qt::LeftButton) {
            RS_CoordinateEvent ce(snapPoint(e));
            coordinateEvent(&ce);
        } else if (e->button()==Qt::RightButton) {
            deletePreview();

            for (RS_Entity* e: addList) {
                container->removeEntity(e);
            }
            addList.clear();
            entity->setSelected(true);
            RS_ActionBlocksExplode* blockexplode  = new RS_ActionBlocksExplode(*container, *graphicView);
            graphicView->redraw(RS2::RedrawDrawing);

            init(getStatus()-1);
        }
    }
}


void sx_arrayData::coordinateEvent(RS_CoordinateEvent* e) {
    if (!e) return;
    RS_Vector mouse = e->getCoordinate();
    switch (getStatus()) {
    case SetCorner1:   //左上角的点
        onePointX = e->getCoordinate().x;
        onePointY = e->getCoordinate().y;
        pPoints->corner1 = mouse;
        graphicView->moveRelativeZero(mouse);
        setStatus(SetCorner2);
        break;
    case SetCorner2:
        pPoints->corner2 = mouse;
        trigger();
        setStatus(SetCorner1);
       // graphicView->setMouseCursor(RS2::WaitCursor);
        break;
    default:
        break;
    }
}


void sx_arrayData::updateMouseCursor() {
    if(openEvent == true)
    {
        graphicView->setMouseCursor(RS2::CadCursor);
    }
}


void sx_arrayData::receiveDataSlot(bool isMouse,bool checkBox,double line1,double line2)
{
    openEvent = isMouse;
    isDeleteEntity = checkBox;
    spacingRow = line1;
    spacingColumn = line2;
    if (graphic) {     //事先定义成一个群组
        RS_BlockList* blockList = graphic->getBlockList();
        if (blockList) {
            RS_BlockData d =
                    RS_DIALOGFACTORY->requestNewGroupDialog(blockList);
            if (!d.name.isEmpty()) {
                RS_Creation creation(container, graphicView);
                creation.createGroup(&d, {10,10}, true);
                RS_InsertData id(
                            d.name,
                            {10,10},
                            RS_Vector(1.0,1.0),
                            0.0,
                            1, 1, RS_Vector(0.0,0.0)
                            );
                creation.createInsert(&id)->setSelected(true);
            }
        }
    }
    for(auto ec: container->getEntityList())
    {
        if(ec->isSelected())
        {
            if(ec->rtti() == RS2::EntityInsert)
            {
                entity = ec;
                entity->setSelected(false);
            }
        }
    }
}

void sx_arrayData::receiveCloseSlot(bool isClose)
{
    if(isClose == true)
    {
        finish(false);
        interArray->close();
    }
}
