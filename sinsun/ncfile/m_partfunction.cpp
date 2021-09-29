#include "m_partfunction.h"

M_PartFunction::M_PartFunction()
{

}

MncSolidData M_PartFunction::caculate_left_refpoints()
{
    //返回指针，直接对指针进行修改
    auto tempPtr=m_solid.m_part;

    if(tempPtr->rtti()==RS2::EntityCircle)
    {
        RS_Vector o = tempPtr->getCenter();
        double r = tempPtr->getRadius();
        RS_Vector CirclePoint(o.x-r,o.y);
        m_solid.m_cutin_point_inPart=CirclePoint;
    }
    else if(tempPtr->rtti()==RS2::EntityPolyline)
    {
        auto b=dynamic_cast<RS_Polyline*>(tempPtr);
        auto allPoints=b->getRefPoints();
        //寻找y值最小的点
        double minX=__DBL_MAX__;
        RS_Vector resultVector(true);

        for(auto tempVec:allPoints)
        {
            if(tempVec.x<minX)
            {
                resultVector=tempVec;
                minX=tempVec.x;
            }
            else if(tempVec.x==minX)
            {
                if(tempVec.y<resultVector.y)
                {
                    resultVector=tempVec;
                }
            }
        }
        m_solid.m_cutin_point_inPart=resultVector;
    }
    else if(tempPtr->rtti()==RS2::EntityEllipse)
    {
        auto b=dynamic_cast<RS_Ellipse*>(tempPtr);

        RS_Vector minCoordinate=tempPtr->getMin();//获取最小值
        RS_Vector maxCoordinate=tempPtr->getMax();
        RS_Line tempLine(RS_Vector(minCoordinate.x,minCoordinate.y),RS_Vector(minCoordinate.x,maxCoordinate.y));
        auto tempSolution=RS_Information::getIntersectionEllipseLine(&tempLine,b);
        //寻找y值最小的点
        double minY=__DBL_MAX__;
        RS_Vector resultVector(true);
        for(auto tempVec:tempSolution)
        {
            if(tempVec.y<minY)
            {
                resultVector=tempVec;
                minY=tempVec.y;
            }
        }
        m_solid.m_cutin_point_inPart=resultVector;
    }

    return m_solid;
}

MncSolidData M_PartFunction::caculate_long_middle_refpoints()
{
    //返回指针，直接对指针进行修改
    auto tempPtr=m_solid.m_part;

    if(tempPtr->rtti()==RS2::EntityCircle)
    {
        RS_Vector o = tempPtr->getCenter();
        double r = tempPtr->getRadius();
        RS_Vector CirclePoint(o.x-r,o.y);

        m_solid.m_cutin_point_inPart=CirclePoint;
    }
    else if(tempPtr->rtti()==RS2::EntityPolyline)
    {
        auto b=dynamic_cast<RS_Polyline*>(tempPtr);
        auto allEntities=b->getEntityList();

        //长边中点
        double maxLength=__DBL_MIN__;
        RS_Vector resultVector(true);
        RS_Line* tempLine;

        for(auto tempEntity:allEntities)
        {
            if(tempEntity->rtti()==RS2::EntityLine&&tempEntity->getLength()>maxLength)
            {
                resultVector=tempEntity->getMiddlePoint();
                maxLength=tempEntity->getLength();
                tempLine=dynamic_cast<RS_Line*>(tempEntity);
            }
        }
        m_solid.m_cutin_point_inPart=resultVector;
    }
    else if(tempPtr->rtti()==RS2::EntityEllipse)
    {
        auto b=dynamic_cast<RS_Ellipse*>(tempPtr);

        RS_Vector minCoordinate=tempPtr->getMin();//获取最小值
        RS_Vector maxCoordinate=tempPtr->getMax();
        RS_Line tempLine(RS_Vector(minCoordinate.x,minCoordinate.y),RS_Vector(minCoordinate.x,maxCoordinate.y));
        auto tempSolution=RS_Information::getIntersectionEllipseLine(&tempLine,b);
        //寻找y值最小的点
        double minY=__DBL_MAX__;
        RS_Vector resultVector(true);
        for(auto tempVec:tempSolution)
        {
            if(tempVec.y<minY)
            {
                resultVector=tempVec;
                minY=tempVec.y;
            }
        }
        m_solid.m_cutin_point_inPart=resultVector;
    }

    return m_solid;
}

MncSolidData M_PartFunction::caculate_coner_refpoints()
{
    //返回指针，直接对指针进行修改
    auto tempPtr=m_solid.m_part;

    if(tempPtr->rtti()==RS2::EntityPolyline)
    {
        auto b=dynamic_cast<RS_Polyline*>(tempPtr);
        m_solid.m_cutin_point_inPart=b->getRefPoints()[0];
    }

    return m_solid;
}

MncSolidData M_PartFunction::caculate_proportion_refpoints(double proporte)
{
    Q_ASSERT(proporte>=0&&proporte<=1);

    //返回指针，直接对指针进行修改
    auto tempPtr=m_solid.m_part;

    if(tempPtr->rtti()==RS2::EntityPolyline)
    {
        auto b=dynamic_cast<RS_Polyline*>(tempPtr);
        m_solid.m_cutin_point_inPart=b->getRefPoints()[0];
    }
    else if(tempPtr->rtti()==RS2::EntityCircle)
    {
        auto b=dynamic_cast<RS_Circle*>(tempPtr);
//        m_solid.m_cutin_point=b->get_proportional_position(m_solid.m_cutin_point,proporte);
    }
    //m_solid.m_cutin_point=b->getRefPoints()[0];


    return m_solid;
}

vector<shared_ptr<RS_Entity>> M_PartFunction::caculate_data_segmentation(bool direction)
{
    vector<shared_ptr<RS_Entity>> result;

    //返回指针，直接对指针进行修改
    RS_Entity* entityPtr=m_solid.m_part;
    RS_Vector cutInPoint=m_solid.m_cutin_point_inPart;
    set<RS_Vector> coldPoints=m_solid.m_cold_points;

    //存入引入线
    if(m_solid.m_in_transition_line.m_is_exist)
    {
        auto inLines=m_solid.m_in_transition_line.m_guide_entities;
        for(auto it:inLines)
        {
            shared_ptr<RS_Entity> tempEntity(it->clone());
            result.push_back(tempEntity);
        }
    }

    //TODO:存入冷却点
    if(entityPtr->rtti()==RS2::EntityPolyline)
    {
        auto tempPolyline=dynamic_cast<RS_Polyline*>(entityPtr);
        auto entityList=tempPolyline->getEntityList();

        //重新拷贝方便后续处理
        vector<shared_ptr<RS_Entity>> tempSharedVector;
        for(int i=0;i<entityList.size();++i)
        {
            shared_ptr<RS_Entity> tempEntity(entityList[i]->clone());
            tempSharedVector.push_back(tempEntity);
        }


        //判断属于哪一个图形
        int special=0;
        for(;special<entityList.size();++special)
        {
            auto tempNearest=entityList[special]->getNearestPointOnEntity(cutInPoint);
            if(tempNearest==cutInPoint)
            {
                break;
            }
        }

        //分割
        auto tempStart=entityList[special]->getStartpoint();
        auto tempEnd=entityList[special]->getEndpoint();

        if(tempStart==cutInPoint)
        {
            result.insert(result.end(),tempSharedVector.begin()+special,tempSharedVector.end());
            result.insert(result.end(),tempSharedVector.begin(),tempSharedVector.begin()+special);
        }
        else if(tempEnd==cutInPoint)
        {
            result.insert(result.end(),tempSharedVector.begin()+special+1,tempSharedVector.end());
            result.insert(result.end(),tempSharedVector.begin(),tempSharedVector.begin()+special+1);
        }
        else
        {
            if(tempSharedVector[special]->rtti()==RS2::EntityLine)
            {
                shared_ptr<RS_Line> tempEntity(new RS_Line(tempStart,cutInPoint));
                result.push_back(tempEntity);
                shared_ptr<RS_Line> tempEntity2(new RS_Line(cutInPoint,tempEnd));
                result.push_back(tempEntity2);
            }
            else if(tempSharedVector[special]->rtti()==RS2::EntityArc)
            {
                shared_ptr<RS_Arc> tempArcPtr = std::dynamic_pointer_cast<RS_Arc>(tempSharedVector[special]);
                shared_ptr<RS_Arc> tempEntity(new RS_Arc(nullptr,RS_ArcData(tempArcPtr->getCenter(),tempArcPtr->getRadius(),tempArcPtr->getAngle1(),tempArcPtr->getCenter().angleTo(cutInPoint),tempArcPtr->isReversed())));
                result.push_back(tempEntity);
                shared_ptr<RS_Arc> tempEntity2(new RS_Arc(nullptr,RS_ArcData(tempArcPtr->getCenter(),tempArcPtr->getRadius(),tempArcPtr->getCenter().angleTo(cutInPoint),tempArcPtr->getAngle2(),tempArcPtr->isReversed())));
                result.push_back(tempEntity2);
            }

            result.insert(result.end(),tempSharedVector.begin()+special+1,tempSharedVector.end());
            result.insert(result.end(),tempSharedVector.begin(),tempSharedVector.begin()+special);
        }
    }
    else if(entityPtr->rtti()==RS2::EntityCircle)
    {
//        //根据冷却点将圆数据分为若干部分
//        //判断切入点与普通冷却点是否相等
//        for(auto it:coldPoints)
//        {
//            if(cutInPoint==it)
//            {
//                shared_ptr<RS_Point> tempPoint(new RS_Point(nullptr,RS_PointData(cutInPoint)));
//                result.push_back(tempPoint);
//                if(coldPoints.size()==1)
//                {
//                    shared_ptr<RS_Circle> tempCircle(tempPtr->clone());
//                    result.push_back(tempCircle);
//                    return result;
//                }
//            }
//        }
        shared_ptr<RS_Entity> tempEntity(entityPtr->clone());
        result.push_back(tempEntity);
    }

    //存入引出线
    if(m_solid.m_out_transition_line.m_is_exist)
    {
        auto outLines=m_solid.m_out_transition_line.m_guide_entities;
        for(auto it:outLines)
        {
            shared_ptr<RS_Entity> tempEntity(it->clone());
            result.push_back(tempEntity);
        }
    }

    return result;
}

vector<RS_Vector> M_PartFunction::get_all_rsvector()
{
    vector<RS_Vector> result;
    if(m_solid.m_part->rtti()==RS2::EntityPolyline)
    {
        auto tempPolyline=dynamic_cast<RS_Polyline*>(m_solid.m_part);
        auto tempList=tempPolyline->getEntityList();

        for(auto it:tempList)
        {
            result.push_back(it->getStartpoint());
        }
    }

    return result;
}
