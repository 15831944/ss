#include "sx_deleterepeat.h"
#include "math.h"


deleteRepeat::deleteRepeat(RS_EntityContainer& container,
                           RS_GraphicView& graphicView)
    :RS_PreviewActionInterface("Blocks Create",container, graphicView)
    ,referencePoint(new RS_Vector{})
{
    point1 = new RS_Vector;
    point2 = new RS_Vector;
    int t = 3;
    int max = 0,min = 0;
    entityList.clear();
    // line = new RS_Line;

    for(auto ec: container.getEntityList())
    {
        if(ec->isSelected())
        {
            entityList.push_back(ec);
        }
    }


    for(auto ec: container.getEntityList())
    {
        if(ec->isSelected())
        {
            ec->setSelected(false);
            for(auto ec2: container.getEntityList())
            {
                if(ec2->isSelected())
                {
                    //qDebug() << "只执行了一次吗";
                    if((ec->rtti() == RS2::EntityCircle) && (ec2->rtti() == RS2::EntityCircle))
                    {
                        if(((ec->getCenter().x == ec2->getCenter().x)&&(ec->getCenter().y == ec2->getCenter().y))&&(ec->getRadius() == ec2->getRadius()))
                        {
                            container.removeEntity(ec2);
                            entityList.removeOne(ec2);

                            continue;
                        }
                    }
                    if((ec->getSize().x >= ec2->getSize().x)&&(ec->getSize().y >= ec2->getSize().y))
                    {
                        if(ec->isPointOnEntity({ec2->getStartpoint().x,ec2->getStartpoint().y}) &&
                                ec->isPointOnEntity({ec2->getEndpoint().x,ec2->getEndpoint().y}))   //起始和结束都在实体上
                        {

                            if((ec->rtti() == RS2::EntityCircle)&&(ec2->rtti() == RS2::EntityArc))
                            {
                                container.removeEntity(ec2);
                                entityList.removeOne(ec2);
                                continue;
                            }
                            if((ec->rtti() != RS2::EntityCircle)&&(ec2->rtti() == RS2::EntityArc))
                            {
                                continue;
                            }
                            container.removeEntity(ec2);
                            entityList.removeOne(ec2);
                        }
                    }
                    else
                    {

                        if(ec2->isPointOnEntity({ec->getStartpoint().x,ec->getStartpoint().y}) &&
                                ec2->isPointOnEntity({ec->getEndpoint().x,ec->getEndpoint().y}))
                        {

                            if((ec2->rtti() == RS2::EntityCircle)&&(ec->rtti() == RS2::EntityArc))
                            {
                                container.removeEntity(ec);
                                entityList.removeOne(ec);
                                continue;
                            }
                            if((ec2->rtti() != RS2::EntityCircle)&&(ec->rtti() == RS2::EntityArc))
                            {
                                continue;
                            }
                            container.removeEntity(ec);
                            entityList.removeOne(ec);
                        }
                    }
                }
            }
        }
    }

    qDebug() << "实体列表大小:" <<entityList.size();
    for(int i = 0;i<entityList.size();i++)
    {
        entityList.at(i)->setSelected(true);
    }



    for(int i = 0;i<entityList.size();i++)
    {
        ec = entityList.at(i);
        t = 3;
        while(t)  //获得实体列表的时候，不知道为什么每次都会少一个，在这里人为做一次循环，遍历一次主实体，就会遍历3次待删除实体
        {
            for(auto ec2: container.getEntityList())
            {
                if(ec2->isSelected())
                {
                    //                qDebug() << "前测试开始" << ec2->getStartpoint().x << ec2->getStartpoint().y;
                    //                qDebug() << "前测试结束" << ec2->getEndpoint().x << ec2->getEndpoint().y;
                    if((ec->rtti() == ec2->rtti())&&(ec->getSize() == ec2->getSize())&&(ec->getStartpoint() == ec2->getStartpoint())
                            &&(ec->getEndpoint() == ec2->getEndpoint()))
                    {
                        continue;
                    }
                    //                qDebug() << "后测试开始" << ec2->getStartpoint().x << ec2->getStartpoint().y;
                    //                qDebug() << "后测试结束" << ec2->getEndpoint().x << ec2->getEndpoint().y;
                    if(ec->isPointOnEntity(ec2->getStartpoint()))   //开始点在实体上
                    {
                        xDifference = ec2->getEndpoint().x - ec2->getStartpoint().x;
                        yDifference = ec2->getEndpoint().y - ec2->getStartpoint().y;
                        if(fabs(xDifference) >= fabs(yDifference))
                        {
                            max = fabs(xDifference);
                            min = fabs(yDifference);
                        }
                        else
                        {
                            max = fabs(yDifference);
                            min = fabs(xDifference);
                        }

                        if(xDifference > 0 && (yDifference == 0))  //结束点大于开始点并且水平
                        {
                            qDebug() << "结束点大于开始点并且水平";
                            for(int i = 0;i <= xDifference;i++)
                            {
                                if(ec->isPointOnEntity({ec2->getStartpoint().x+i,ec2->getStartpoint().y}))
                                {

                                }
                                else
                                {
                                    RS_Vector *vectorList = new RS_Vector({ec2->getStartpoint().x+i,ec2->getStartpoint().y});
                                    pointList.push_back(vectorList);
                                }
                            }
                            RS_Line *line = new RS_Line;
                            pointList.at(0)->x = pointList.at(0)->x - 1;
                            line->setStartpoint(*(pointList.at(0)));
                            line->setEndpoint(*(pointList.at(pointList.size()-1)));
                            pointList.clear();
                            container.removeEntity(ec2);
                            entityList.removeOne(ec2);
                            container.addEntity(line);

                        }
                        if(xDifference < 0 && (yDifference == 0))
                        {
                            xDifference = xDifference * (-1);
                            qDebug() << "开始点大于结束点且水平";
                            for(int i = 0;i <= xDifference;i++)
                            {
                                if(ec->isPointOnEntity({ec2->getStartpoint().x-i,ec2->getStartpoint().y}))
                                {

                                }
                                else
                                {
                                    RS_Vector *vectorList = new RS_Vector({ec2->getStartpoint().x-i,ec2->getStartpoint().y});
                                    pointList.push_back(vectorList);
                                }
                            }

                            RS_Line *line = new RS_Line;
                            pointList.at(0)->x = pointList.at(0)->x + 1;
                            line->setStartpoint(*(pointList.at(0)));
                            line->setEndpoint(*(pointList.at(pointList.size()-1)));
                            pointList.clear();
                            qDebug() << "开始:" <<line->getStartpoint().x << line->getStartpoint().y;
                            qDebug() << "结束:" <<line->getEndpoint().x << line->getEndpoint().y;
                            container.removeEntity(ec2);
                            entityList.removeOne(ec2);

                            container.addEntity(line);
                        }

                        if(xDifference > 0 && yDifference > 0)  //既不水平也不垂直且x,y的结束都大于开始
                        {
                            int j = 0;
                            for(int i = 0;i <= max;i++)
                            {
                                if(j >= min)
                                {
                                    j = min;
                                }
                                if(fabs(xDifference) >= fabs(yDifference))  //x轴坐标大于y轴坐标
                                {
                                    if(ec->isPointOnEntity({ec2->getStartpoint().x+i,ec2->getStartpoint().y+j}))
                                    {

                                    }
                                    else
                                    {
                                        RS_Vector *vectorList = new RS_Vector({ec2->getStartpoint().x+i,ec2->getStartpoint().y+j});
                                        pointList.push_back(vectorList);
                                    }
                                }
                                else   //y轴大于x轴
                                {
                                    if(ec->isPointOnEntity({ec2->getStartpoint().x+j,ec2->getStartpoint().y+i}))
                                    {
                                        qDebug() << "在实体上的点:" <<ec2->getStartpoint().x << ec2->getStartpoint().y;
                                    }
                                    else
                                    {
                                        RS_Vector *vectorList = new RS_Vector({ec2->getStartpoint().x+j,ec2->getStartpoint().y+i});
                                        pointList.push_back(vectorList);
                                    }
                                }
                                j++;
                            }
                            RS_Line *line = new RS_Line;
                            pointList.at(0)->x = pointList.at(0)->x - 1;
                            pointList.at(0)->y = pointList.at(0)->y - 1;
                            line->setStartpoint(*(pointList.at(0)));
                            line->setEndpoint(*(pointList.at(pointList.size()-1)));
                            qDebug() << "结束大于开始:开始:" <<line->getStartpoint().x << line->getStartpoint().y;
                            qDebug() << "结束大于开始:结束" <<line->getEndpoint().x << line->getEndpoint().y;
                            pointList.clear();
                            container.removeEntity(ec2);
                            entityList.removeOne(ec2);
                            container.addEntity(line);
                        }
                        if(xDifference > 0 && yDifference < 0)  //既不水平也不垂直且x的结束大于开始，y的开始大于结束
                        {
                            int j = 0;
                            for(int i = 0;i <= max;i++)
                            {
                                if(j >= min)
                                {
                                    j = min;
                                }
                                if(fabs(xDifference) >= fabs(yDifference))  //x轴坐标大于y轴坐标
                                {
                                    if(ec->isPointOnEntity({ec2->getStartpoint().x+i,ec2->getStartpoint().y-j}))
                                    {
                                        qDebug() << "249行";
                                    }
                                    else
                                    {
                                        RS_Vector *vectorList = new RS_Vector({ec2->getStartpoint().x+i,ec2->getStartpoint().y-j});
                                        pointList.push_back(vectorList);
                                    }
                                }
                                else   //y轴大于x轴
                                {
                                    if(ec->isPointOnEntity({ec2->getStartpoint().x-j,ec2->getStartpoint().y+i}))
                                    {
                                        qDebug() << "在实体上的点:" <<ec2->getStartpoint().x << ec2->getStartpoint().y;
                                    }
                                    else
                                    {
                                        RS_Vector *vectorList = new RS_Vector({ec2->getStartpoint().x-j,ec2->getStartpoint().y+i});
                                        pointList.push_back(vectorList);
                                    }
                                }
                                j++;
                            }
                            RS_Line *line = new RS_Line;
                            pointList.at(0)->x = pointList.at(0)->x - 1;
                            pointList.at(0)->y = pointList.at(0)->y -1 ;
                            line->setStartpoint(*(pointList.at(0)));
                            line->setEndpoint(*(pointList.at(pointList.size()-1)));
                            qDebug() << "开始:" <<line->getStartpoint().x << line->getStartpoint().y;
                            qDebug() << "结束:" <<line->getEndpoint().x << line->getEndpoint().y;
                            pointList.clear();
                            container.removeEntity(ec2);
                            entityList.removeOne(ec2);
                            container.addEntity(line);
                        }

                        if(xDifference < 0 && yDifference > 0)  //既不水平也不垂直且x,y的结束都大于开始
                        {
                            int j = 0;
                            for(int i = 0;i <= max;i++)
                            {
                                if(j >= min)
                                {
                                    j = min;
                                }
                                if(fabs(xDifference) >= fabs(yDifference))  //x轴坐标大于y轴坐标
                                {
                                    if(ec->isPointOnEntity({ec2->getStartpoint().x-i,ec2->getStartpoint().y+j}))
                                    {

                                    }
                                    else
                                    {
                                        RS_Vector *vectorList = new RS_Vector({ec2->getStartpoint().x-i,ec2->getStartpoint().y+j});
                                        pointList.push_back(vectorList);
                                    }
                                }
                                else   //y轴大于x轴
                                {
                                    if(ec->isPointOnEntity({ec2->getStartpoint().x-j,ec2->getStartpoint().y+i}))
                                    {
                                        qDebug() << "在实体上的点:" <<ec2->getStartpoint().x << ec2->getStartpoint().y;
                                    }
                                    else
                                    {
                                        RS_Vector *vectorList = new RS_Vector({ec2->getStartpoint().x-j,ec2->getStartpoint().y+i});
                                        pointList.push_back(vectorList);
                                    }
                                }
                                j++;
                            }
                            RS_Line *line = new RS_Line;
                            pointList.at(0)->x = pointList.at(0)->x - 1;
                            pointList.at(0)->y = pointList.at(0)->y;
                            line->setStartpoint(*(pointList.at(0)));
                            line->setEndpoint(*(pointList.at(pointList.size()-1)));
                            qDebug() << "开始:" <<line->getStartpoint().x << line->getStartpoint().y;
                            qDebug() << "结束:" <<line->getEndpoint().x << line->getEndpoint().y;
                            pointList.clear();
                            container.removeEntity(ec2);
                            entityList.removeOne(ec2);
                            container.addEntity(line);
                        }
                        if(xDifference < 0 && yDifference < 0)  //既不水平也不垂直且x,y的结束都大于开始
                        {
                            int j = 0;
                            for(int i = 0;i <= max;i++)
                            {
                                if(j >= min)
                                {
                                    j = min;
                                }
                                if(fabs(xDifference) >= fabs(yDifference))  //x轴坐标大于y轴坐标
                                {
                                    if(ec->isPointOnEntity({ec2->getStartpoint().x-i,ec2->getStartpoint().y-j}))
                                    {

                                    }
                                    else
                                    {
                                        RS_Vector *vectorList = new RS_Vector({ec2->getStartpoint().x-i,ec2->getStartpoint().y-j});
                                        pointList.push_back(vectorList);
                                    }
                                }
                                else   //y轴大于x轴
                                {
                                    if(ec->isPointOnEntity({ec2->getStartpoint().x-j,ec2->getStartpoint().y-i}))
                                    {
                                        qDebug() << "在实体上的点:" <<ec2->getStartpoint().x << ec2->getStartpoint().y;
                                    }
                                    else
                                    {
                                        RS_Vector *vectorList = new RS_Vector({ec2->getStartpoint().x-j,ec2->getStartpoint().y-i});
                                        pointList.push_back(vectorList);
                                    }
                                }
                                j++;
                            }
                            RS_Line *line = new RS_Line;
                            pointList.at(0)->x = pointList.at(0)->x + 1;
                            pointList.at(0)->y = pointList.at(0)->y + 1;
                            line->setStartpoint(*(pointList.at(0)));
                            line->setEndpoint(*(pointList.at(pointList.size()-1)));
                            qDebug() << "开始大于结束:开始:" <<line->getStartpoint().x << line->getStartpoint().y;
                            qDebug() << "开始大于结束:结束:" <<line->getEndpoint().x << line->getEndpoint().y;
                            pointList.clear();
                            container.removeEntity(ec2);
                            entityList.removeOne(ec2);
                            container.addEntity(line);
                        }



                    }
                }

            }
            qDebug() << "---------内循环结束----------";
            t--;
        }
    }
    graphicView.redraw(RS2::RedrawDrawing);
    container.setSelected(false);
    finish(false);
}
