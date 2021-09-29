#include "rs_actionmergenew.h"
#include<QDebug>

rs_actionmergenew::rs_actionmergenew(RS_EntityContainer &container, RS_GraphicView &graphicView) :
    RS_ActionInterface("WeiLian Entities", container, graphicView){
    am = new rs_actionmerge();
    connect(am,SIGNAL(myMergeSignal(double)),this,SLOT(myMergeSignal(double)));
    connect(am,SIGNAL(closeWindowSignal(bool)),this,SLOT(receiveCloseSlot(bool)));
    am->exec();
}

void rs_actionmergenew::myMergeSignal(double data)
{
    std::vector<RS_Entity*> addList;
    std::vector<RS_Line*> newList;
    std::vector<RS_Vector> vectorList;
    std::vector<RS_Entity*> blockList;
    RS_BlockList* blockList1 = graphicView->getGraphic()->getBlockList();

    double mindistance = 101;
    RS_Vector minv;
    RS_Vector flag;
    std::vector<RS_Entity*>::iterator k;
    std::vector<RS_Entity*>::iterator d;
    std::vector<RS_Vector>::iterator j;
    for(auto ec: container->getEntityList()){
        if(ec->rtti()==RS2::EntityArc||ec->rtti()==RS2::EntityLine){
            if(ec->isSelected()){
                addList.push_back(ec);
                blockList.push_back(ec);
                vectorList.push_back(ec->getStartpoint());
                vectorList.push_back(ec->getEndpoint());
            }
        }
    }



    for (RS_Entity* e: addList) {
        mindistance = 101;
        qDebug()<<addList.size();
        qDebug()<<vectorList.size();
        /*从实体容器中删除实体*/
        k = find(addList.begin(), addList.end(),e);

        if(k == addList.end()) //要控制迭代器不能超过整个容器
        {
            break;
        }

        addList.erase(k);
        /*如果实体的两端点都在断点容器中*/
        if(count(vectorList.begin(),vectorList.end(),e->getStartpoint())&&count(vectorList.begin(),vectorList.end(),e->getEndpoint())){
            qDebug()<<"2222222";
            /*从端点容器中删除当前实体两端点*/
            j = find(vectorList.begin(), vectorList.end(),e->getStartpoint());
            vectorList.erase(j);
            j = find(vectorList.begin(), vectorList.end(),e->getEndpoint());
            vectorList.erase(j);
            /*如果删除两端点后，端点容器为空则退出循环*/
            if(vectorList.size()==0){
              /*  d = find(blockList.begin(),blockList.end(),e);
                blockList.erase(d);*/
                e->setSelected(0);
                break;
            }
            /*找出两端点距离端点容器中各点的最短距离*/
            for(RS_Vector ec: vectorList){
                if(e->getEndpoint().distanceTo(ec)<mindistance){
                    mindistance = e->getEndpoint().distanceTo(ec);
                    flag = e->getEndpoint();
                    minv = ec;
                }
                if(e->getStartpoint().distanceTo(ec)<mindistance){
                    mindistance = e->getStartpoint().distanceTo(ec);
                    flag = e->getStartpoint();
                    minv = ec;
                }
            }

            /*如果最短距离是末端点的，画出直线，再找起始点的最短距离，画出直线*/
            if(flag==e->getEndpoint()){
                if(mindistance<=data){
                    line = new RS_Line(e->getEndpoint(),minv);
                    newList.push_back(line);
                    blockList.push_back(line);
                    line->setSelected(1);
                    j = find(vectorList.begin(), vectorList.end(),minv);
                    vectorList.erase(j);
                }
                mindistance = 101;
                for(RS_Vector ec: vectorList){
                    if(e->getStartpoint().distanceTo(ec)<mindistance){
                        mindistance = e->getStartpoint().distanceTo(ec);
                        minv = ec;
                    }
                }
                if(mindistance<=data){
                    line = new RS_Line(e->getStartpoint(),minv);
                    blockList.push_back(line);
                    newList.push_back(line);
                    line->setSelected(1);
                    j = find(vectorList.begin(), vectorList.end(),minv);
                    vectorList.erase(j);
                }
                /*如果最短距离是起始点的，画出直线，再找末端点的最短距离，画出直线*/
            }else{
                if(mindistance<=data){
                    line = new RS_Line(e->getStartpoint(),minv);
                    blockList.push_back(line);
                    newList.push_back(line);
                    line->setSelected(1);
                    j = find(vectorList.begin(), vectorList.end(),minv);
                    vectorList.erase(j);
                }
                mindistance = 101;
                for(RS_Vector ec: vectorList){
                    if(e->getEndpoint().distanceTo(ec)<mindistance){
                        mindistance = e->getEndpoint().distanceTo(ec);
                        minv = ec;
                    }
                }
                if(mindistance<=data){
                    line = new RS_Line(e->getEndpoint(),minv);
                    blockList.push_back(line);
                    newList.push_back(line);
                    line->setSelected(1);
                    j = find(vectorList.begin(), vectorList.end(),minv);
                    vectorList.erase(j);
                }

            }

            //如果只有一个端点在端点容器中
        }else{
            qDebug()<<"111111111";
            if(count(vectorList.begin(),vectorList.end(),e->getStartpoint())){
                j = find(vectorList.begin(), vectorList.end(),e->getStartpoint());
                vectorList.erase(j);
                if(vectorList.size()==0){
                   /* d = find(blockList.begin(),blockList.end(),e);
                    blockList.erase(d);*/
                    e->setSelected(0);
                    break;
                }
                for(RS_Vector ec: vectorList){
                    if(e->getStartpoint().distanceTo(ec)<mindistance){
                        mindistance = e->getStartpoint().distanceTo(ec);
                        minv = ec;
                    }
                }
                if(mindistance<=data){
                    line = new RS_Line(e->getStartpoint(),minv);
                    blockList.push_back(line);
                    newList.push_back(line);
                    line->setSelected(1);
                    j = find(vectorList.begin(), vectorList.end(),minv);
                    vectorList.erase(j);
                }
            }else{
                j = find(vectorList.begin(), vectorList.end(),e->getEndpoint());
                vectorList.erase(j);
                if(vectorList.size()==0){
                    /*d = find(blockList.begin(),blockList.end(),e);
                    blockList.erase(d);*/
                    e->setSelected(0);
                    break;
                }
                for(RS_Vector ec: vectorList){
                    if(e->getEndpoint().distanceTo(ec)<mindistance){
                        mindistance = e->getEndpoint().distanceTo(ec);
                        minv = ec;
                    }
                }
                if(mindistance<=data){
                    line = new RS_Line(e->getEndpoint(),minv);
                    blockList.push_back(line);
                    newList.push_back(line);
                    line->setSelected(1);
                    j = find(vectorList.begin(), vectorList.end(),minv);
                    vectorList.erase(j);
                }
            }
        }
        if(addList.size()==0||vectorList.size()==0)
            break;
    }


  //  qDebug()<<newList.size();
    for (RS_Entity* e: newList) {
        graphicView->drawEntity(e);
        container->addEntity(e);
        //e->setSelected(1);
    }
    if (blockList1) {
                RS_BlockData d;
                d.name = "1";
                   /* RS_DIALOGFACTORY->requestNewBlockDialog(blockList1);*/
                if (!d.name.isEmpty()) {
                    RS_Creation creation(container, graphicView);
                    creation.createBlock(&d, d.basePoint, true);
                    RS_InsertData id(
                        d.name,
                        d.basePoint,
                        RS_Vector(1.0,1.0),
                        0.0,
                        1, 1, RS_Vector(0.0,0.0)
                    );
                    creation.createInsert(&id);
                }

            }
    graphicView->redraw(RS2::RedrawDrawing);
    finish();
}

void rs_actionmergenew::receiveCloseSlot(bool isClose)
{
    if(isClose == true)
        {
            finish(false);
            am->close();
        }
}


