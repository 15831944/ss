#include "pt_algcommon.h"
#include "rs_vector.h"
#include "rs_line.h"
#include "rs_graphicview.h"
#include "qc_mdiwindow.h"
#include "lc_simpletests.h"
#include "rs_block.h"
#include "rs_graphic.h"
#include "rs_arc.h"
#include "rs_circle.h"
#include "rs_insert.h"
#include "rs_math.h"
#include "rs_ellipse.h"
#include "rs_polyline.h"
#include "rs_information.h"

bool PtCommonFunctions::read_num_after_str(const char *buff, const char *target, double &ans)
{
    if (strlen(target)==0|| strlen(buff)==0)
    {
        return false;
    }
    else
    {
        //使用string的方式查找
        std::string  str(buff);
        std::string  temp_str;

        size_t start_station = str.find(target) + strlen(target);//查找目标位置,加上char*长度
        size_t end_station=start_station;
        for(;str[end_station]!='\0';++end_station)
        {
            if((str[end_station]>'A'&&str[end_station]<'Z')||str[end_station]==' ')
            {
                break;
            }
        }
        temp_str = str.substr(start_station, end_station);
        ans = atof(temp_str.data());                            //将字符串转为double类型
        return true;
    }
}

map<int,RS_Entity*> PtCommonFunctions::get_selected_entities(const QList<RS_Entity *> val)
{
    map<int,RS_Entity*> result;
    for(size_t i=0;i<val.size();++i)
    {
        if(val[i]->isSelected()==true)
        {
            result[i]=val[i];
        }
    }
    return result;
}

shared_ptr<RS_Polyline> PtCommonFunctions::ptcopy_polyline_deeply(RS_Polyline *in)
{
    shared_ptr<RS_Polyline> tempPolyLine(new RS_Polyline(nullptr,in->getData()));

    QList<RS_Entity*> list=in->getEntityList();

    for(int i=0;i<list.size();++i)
    {
        if(list[i]->rtti()==RS2::EntityLine)
        {
            auto temp=dynamic_cast<RS_Line*>(list[i]);
            RS_Line* newLine=new RS_Line(nullptr,temp->getData());
//            tempPolyLine->getEntityList(1).push_back(newLine);
        }
        else if(list[i]->rtti()==RS2::EntityArc)
        {
            auto temp=dynamic_cast<RS_Arc*>(list[i]);
            RS_Arc* newLine=new RS_Arc(nullptr,temp->getData());
//            tempPolyLine->getEntityList(1).push_back(newLine);
        }
    }

    return tempPolyLine;
}

RS_Polyline *PtCommonFunctions::ptcopy_polyline_deeply(shared_ptr<RS_Polyline> in)
{
    RS_Polyline* tempPolyLine=new RS_Polyline(nullptr,in->getData());

    QList<RS_Entity*> list=in->getEntityList();

    for(int i=0;i<list.size();++i)
    {
        if(list[i]->rtti()==RS2::EntityLine)
        {
            auto temp=dynamic_cast<RS_Line*>(list[i]);
            RS_Line* newLine=new RS_Line(nullptr,temp->getData());
//            tempPolyLine->getEntityList(1).push_back(newLine);
        }
        else if(list[i]->rtti()==RS2::EntityArc)
        {
            auto temp=dynamic_cast<RS_Arc*>(list[i]);
            RS_Arc* newLine=new RS_Arc(nullptr,temp->getData());
//            tempPolyLine->getEntityList(1).push_back(newLine);
        }
    }

    return tempPolyLine;
}
