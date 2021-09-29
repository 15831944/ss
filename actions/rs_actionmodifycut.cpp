/****************************************************************************
**
** This file is part of the LibreCAD project, a 2D CAD program
**
** Copyright (C) 2010 R. van Twisk (librecad@rvt.dds.nl)
** Copyright (C) 2001-2003 RibbonSoft. All rights reserved.
**
**
** This file may be distributed and/or modified under the terms of the
** GNU General Public License version 2 as published by the Free Software
** Foundation and appearing in the file gpl-2.0.txt included in the
** packaging of this file.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with this program; if not, write to the Free Software
** Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
**
** This copyright notice MUST APPEAR in all copies of the script!
**
**********************************************************************/

#include "rs_actionmodifycut.h"

#include <QAction>
#include <QMouseEvent>
#include "rs_dialogfactory.h"
#include "rs_graphicview.h"
#include "rs_modification.h"
#include "rs_debug.h"
#include "QDebug"
#include "sx_actiongroupexplode.h"
#include "rs_polyline.h"
#include "rs_arc.h"
#include "math.h"
#include "rs_commandevent.h"

struct RS_ActionModifyCut::Points {

    /**
     * Line data defined so far.
     */
    RS_PolylineData data;
    RS_PolylineData data2;
    RS_ArcData arc_data;
    /**
     * Polyline entity we're working on.
     */
    RS_Polyline* polyline;
    RS_Polyline* polyline2;
    /**
     * last point.
     */
    RS_Vector point;
    RS_Vector calculatedEndpoint;
    /**
     * Start point of the series of lines. Used for close function.
     */
    RS_Vector start;

    /**
     * Point history (for undo)
     */
    QList<RS_Vector> history;

    /**
     * Bulge history (for undo)
     */
    QList<double> bHistory;
};


RS_ActionModifyCut::RS_ActionModifyCut(RS_EntityContainer& container,
                                       RS_GraphicView& graphicView)
    :RS_ActionInterface("Cut Entity",
                        container, graphicView)
    ,cutEntity(nullptr)
    ,cutCoord(new RS_Vector{})
    , pPoints(new Points{})
{
    actionType=RS2::ActionModifyCut;
    leftCoord = new RS_Vector;
    rightCoord = new RS_Vector;
    startPoint = new RS_Vector;
    endPoint = new RS_Vector;
}

RS_ActionModifyCut::~RS_ActionModifyCut() = default;

void RS_ActionModifyCut::init(int status) {
    RS_ActionInterface::init(status);
}

void RS_ActionModifyCut::trigger() {

    RS_DEBUG->print("RS_ActionModifyCut::trigger()");

    if (cutEntity && cutEntity->isAtomic() && cutCoord->valid &&
            cutEntity->isPointOnEntity(*cutCoord)) {
        cutEntity->setHighlighted(false);
        graphicView->drawEntity(cutEntity);

        RS_Modification m(*container, graphicView);
        m.cut(*cutCoord, (RS_AtomicEntity*)cutEntity);
        if(flag == true)
        {
            container->removeEntity(cutEntity);  //如果不移除这个实体会出错
            pPoints->point = *cutCoord;
            *leftCoord = *cutCoord;
            *rightCoord = *cutCoord;
            pPoints->history.clear();
            pPoints->history.append(*cutCoord);
            pPoints->bHistory.clear();
            pPoints->bHistory.append(0.0);
            pPoints->start = pPoints->point;
            bool exitFactor = false;
            qDebug() << "鼠标点:" << cutCoord->x << " " << cutCoord->y;
            if(isClose == true)  //闭合
            {
                qDebug() << "闭合图形";
                while(1)
                {
                    for(auto ec:container->getEntityList())
                    {
                        if(ec->isPointOnEntity(*rightCoord))
                        {
                            if((rightCoord->x == ec->getStartpoint().x) && (rightCoord->y == ec->getStartpoint().y))
                            {
                                if (!pPoints->polyline) {
                                    pPoints->polyline = new RS_Polyline(container, pPoints->data);
                                    pPoints->polyline->addVertex(pPoints->start, 0.0);
                                }
                                if (pPoints->polyline) {
                                    pPoints->polyline->addVertex({ec->getEndpoint().x,ec->getEndpoint().y}, 0.0);
                                    pPoints->polyline->setEndpoint({ec->getEndpoint().x,ec->getEndpoint().y});
                                    *rightCoord = ec->getEndpoint();
                                }
                                container->removeEntity(ec);
                            }
                        }
                    }
                    if(((rightCoord->x == endPoint->x)&&(rightCoord->y == endPoint->y)))
                    {
                        break;
                    }
                }
                container->addEntity(pPoints->polyline);
                graphicView->drawEntity(pPoints->polyline);
                pPoints->polyline = NULL;
                isClose = false;
            }
            else{
                qDebug() << "非闭合图形";
                /*右半边图形*/
                while(1)
                {
                    for(auto ec:container->getEntityList())
                    {
                        if(ec->isPointOnEntity(*rightCoord))
                        {
                            if((rightCoord->x == ec->getStartpoint().x) && (rightCoord->y == ec->getStartpoint().y))
                            {
                                if (!pPoints->polyline) {
                                    pPoints->polyline = new RS_Polyline(container, pPoints->data);
                                    pPoints->polyline->addVertex(pPoints->start, 0.0);
                                }
                                if (pPoints->polyline) {
                                    pPoints->polyline->addVertex({ec->getEndpoint().x,ec->getEndpoint().y}, 0.0);
                                    pPoints->polyline->setEndpoint({ec->getEndpoint().x,ec->getEndpoint().y});
                                    *rightCoord = ec->getEndpoint();
                                   // qDebug() << "找结束的图形:" << rightCoord->x << rightCoord->y;
                                }
                                container->removeEntity(ec);
                                if(((rightCoord->x == endPoint->x)&&(rightCoord->y == endPoint->y)))
                                {
                                    exitFactor = true;
                                    break;
                                }
                            }
                            else if((rightCoord->x == ec->getEndpoint().x) && (rightCoord->y == ec->getEndpoint().y))
                            {
                                if (!pPoints->polyline) {
                                    pPoints->polyline = new RS_Polyline(container, pPoints->data);
                                    pPoints->polyline->addVertex(pPoints->start, 0.0);
                                }
                                if (pPoints->polyline) {
                                    pPoints->polyline->addVertex({ec->getStartpoint().x,ec->getStartpoint().y}, 0.0);
                                    pPoints->polyline->setEndpoint({ec->getStartpoint().x,ec->getStartpoint().y});
                                    *rightCoord = ec->getStartpoint();
                                   // qDebug() << "找结束的图形222:" << rightCoord->x << rightCoord->y;
                                }
                                container->removeEntity(ec);
                                if(((rightCoord->x == startPoint->x)&&(rightCoord->y == startPoint->y)))
                                {
                                    exitFactor = true;
                                    break;
                                }
                            }
                        }
                    }
                    if(exitFactor == true)
                    {
                        exitFactor = false;
                        break;
                    }
                }
                container->addEntity(pPoints->polyline);
                graphicView->drawEntity(pPoints->polyline);
                pPoints->polyline = NULL;
                //qDebug() << "右半边图形执行完毕";
                /*左半边图形*/
                while(1)
                {
                    for(auto ec:container->getEntityList())
                    {
                        if(ec->isPointOnEntity(*leftCoord))
                        {
                            if((leftCoord->x == ec->getEndpoint().x) && (leftCoord->y == ec->getEndpoint().y))
                            {
                                if (!pPoints->polyline) {
                                    pPoints->polyline = new RS_Polyline(container, pPoints->data);
                                    pPoints->polyline->addVertex(pPoints->start, 0.0);
                                }
                                if (pPoints->polyline) {
                                    pPoints->polyline->addVertex({ec->getStartpoint().x,ec->getStartpoint().y}, 0.0);
                                    pPoints->polyline->setEndpoint({ec->getStartpoint().x,ec->getStartpoint().y});
                                    *leftCoord = ec->getStartpoint();
                                    //qDebug() << "找开始的图形:" << leftCoord->x << leftCoord->y;
                                }
                                container->removeEntity(ec);
                                if(((leftCoord->x == startPoint->x)&&(leftCoord->y == startPoint->y)))
                                {
                                    exitFactor = true;
                                    break;
                                }
                            }
                            else if((leftCoord->x == ec->getStartpoint().x) && (leftCoord->y == ec->getStartpoint().y))
                            {
                                if (!pPoints->polyline) {
                                    pPoints->polyline = new RS_Polyline(container, pPoints->data);
                                    pPoints->polyline->addVertex(pPoints->start, 0.0);
                                }
                                if (pPoints->polyline) {
                                    pPoints->polyline->addVertex({ec->getEndpoint().x,ec->getEndpoint().y}, 0.0);
                                    pPoints->polyline->setEndpoint({ec->getEndpoint().x,ec->getEndpoint().y});
                                    *leftCoord = ec->getEndpoint();
                                    //qDebug() << "找结束的图形:" << leftCoord->x << leftCoord->y;
                                }
                                container->removeEntity(ec);
                                if(((leftCoord->x == endPoint->x)&&(leftCoord->y == endPoint->y)))
                                {
                                    exitFactor = true;
                                    break;
                                }
                            }
                        }

                    }
                    if(exitFactor == true)
                    {
                        exitFactor = false;
                        break;
                    }

                }
                container->addEntity(pPoints->polyline);
                graphicView->drawEntity(pPoints->polyline);
                pPoints->polyline = NULL;
            }
            flag = false;
        }
        cutEntity = nullptr;
        *cutCoord = RS_Vector(false);
        setStatus(ChooseCutEntity);
        RS_DIALOGFACTORY->updateSelectionWidget(container->countSelected(),container->totalSelectedLength());
    }
    else if((cutEntity->rtti() == RS2::EntityPolyline) && cutCoord->valid && cutEntity->isPointOnEntity(*cutCoord))
    {
        container->setSelected(false);
        cutEntity->setSelected(true);
        blockexplode  = new sx_actionGroupExplode(*container, *graphicView);

        for(auto ec:container->getEntityList())
        {
            if(ec->isPointOnEntity(*cutCoord))
            {
                flag = true;
                //*tmpCutCoord = *cutCoord;  //将第一次的坐标点保存
                cutEntity = ec;
            }
        }
        trigger();
    }
}



void RS_ActionModifyCut::mouseMoveEvent(QMouseEvent* e) {
    RS_DEBUG->print("RS_ActionModifyCut::mouseMoveEvent begin");

    switch (getStatus()) {
    case ChooseCutEntity:
        deleteSnapper();
        break;

    case SetCutCoord:
        snapPoint(e);
        break;

    default:
        break;
    }

    RS_DEBUG->print("RS_ActionModifyTrim::mouseMoveEvent end");
}


void RS_ActionModifyCut::mouseReleaseEvent(QMouseEvent* e) {

    if (e->button()==Qt::LeftButton) {
        vector = new RS_Vector(e->x(),e->y());
        RS_Vector pointVector = graphicView->toGraph(*vector);
        for(auto ec:container->getEntityList())
        {
            if(ec->isPointOnEntity(pointVector,0.4))
            {
                *cutCoord = ec->getNearestPointOnEntity(pointVector);
                cutEntity = ec;
                qDebug() << "原本鼠标点:" << pointVector.x << "" << pointVector.y;
                qDebug() << "原本实体点:" << cutCoord->x << "" << cutCoord->y;
                if(((fabs(pointVector.x - cutCoord->x)) > 1)||((fabs(pointVector.y - cutCoord->y)) > 1))
                {
                    pointVector.y = pointVector.y + 0.5;
                    *cutCoord = ec->getNearestPointOnEntity(pointVector);
                    qDebug() << "更改鼠标点:" << pointVector.x << "" << pointVector.y;
                    qDebug() << "更改实体点:" << cutCoord->x << "" << cutCoord->y;
                    //qDebug() << "坐标不对";
                }
                if(ec->rtti() == RS2::EntityPolyline)
                {
                    if(((RS_Polyline*)ec)->isClosed())  //判断是否闭合
                    {
                        *startPoint = *cutCoord;
                        *endPoint = *cutCoord;
                        isClose = true;
                    }
                    else
                    {
                        *startPoint = ec->getStartpoint();
                        *endPoint = ec->getEndpoint();
                        qDebug() << "开始11:" << startPoint->x << startPoint->y <<"" <<"结束11:" << endPoint->x << endPoint->y;
                    }
                }
            }
        }

        //	*cutCoord = snapPoint(e);  //捕捉模式捕捉实体上的点

        if (cutEntity==nullptr) {
            RS_DIALOGFACTORY->commandMessage(tr("No Entity found."));
            return;
        } else if (!cutCoord->valid) {
            RS_DIALOGFACTORY->commandMessage(tr("Cutting point is invalid."));
            return;
        } else if (!cutEntity->isPointOnEntity(*cutCoord)) {
            RS_DIALOGFACTORY->commandMessage(
                        tr("Cutting point is not on entity."));
            return;
        } else {
            trigger();
            deleteSnapper();
        }
    }else if (e->button()==Qt::RightButton) {
        if (cutEntity) {
            cutEntity->setHighlighted(false);
            graphicView->drawEntity(cutEntity);
        }
        init(getStatus()-1);
    }
}



void RS_ActionModifyCut::updateMouseButtonHints() {
    switch (getStatus()) {
    case ChooseCutEntity:
        RS_DIALOGFACTORY->updateMouseWidget(tr("Please click in the split position"),
                                            tr("Cancel"));
        break;
    case SetCutCoord:
        RS_DIALOGFACTORY->updateMouseWidget(tr("Please click in the split position"),
                                            tr("Back"));
        break;
    default:
        RS_DIALOGFACTORY->updateMouseWidget();
        break;
    }
}








void RS_ActionModifyCut::updateMouseCursor()
{
    switch (getStatus()) {
    case ChooseCutEntity:
        graphicView->setMouseCursor(RS2::SplitHCursor);
        break;
    case SetCutCoord:
        graphicView->setMouseCursor(RS2::MovingHandCursor);
        break;
    default:
        break;
    }
}

// EOF
