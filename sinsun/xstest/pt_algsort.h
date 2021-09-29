#ifndef PT_ALGSORT_H
#define PT_ALGSORT_H
#include "rs_document.h"
#include <vector>
#include "QList"
#include "city.h"
#include "algorithms.h"
#include "ncfile.h"

enum POINT_POSITION{PTIN,PTON,PTOUT};

class Pt_AlgSort
{
public:
    Pt_AlgSort();
    virtual ~Pt_AlgSort();

public:
    void set_ori_entities(QList<RS_Entity *> _val){m_oriEntities=_val;}

    void get_after_ncfile(shared_ptr<ncFileData> inData);

    void show_connect_lines(RS_Graphic *_Graphic,std::vector<RS_Vector> _ConnectPoint);

protected:
    QList<RS_Entity *> m_oriEntities;

    vector<int> m_after_sort_sequence;

};

//排列实体
class Pt_AlgSortEntities:public Pt_AlgSort
{
public:
    Pt_AlgSortEntities(){};
    ~Pt_AlgSortEntities(){};
public:
    QList<RS_Entity *> get_after_entities(){return m_afterEntities;}
public:
    //基本排序算法
    void sort_func_basic(std::vector<RS_Vector>& _ConnectPoint);

    //
    void sort_func_grid(std::vector<RS_Vector>& _ConnectPoint);

    QList<RS_Vector> get_after_points();

private:
    QList<RS_Entity *> m_afterEntities;

};



class Pt_CommonFunctions
{
public:
    static void show_connect_lines(RS_Graphic *_Graphic,QList<RS_Entity*> _ConnectPoint,RS_Color color=RS_Color(255, 0, 0))
    {
        if(!_ConnectPoint.empty())
        {
            for(auto it:_ConnectPoint)
            {
                it->setPen(RS_Pen(color,RS2::Width08,RS2::SolidLine));
                _Graphic->addEntity(it);
            }
        }
    }

    static POINT_POSITION point_and_circle_relative(const RS_Vector& _val,const RS_Circle& _circle)
    {
        double distance=_val.distanceTo(_circle.getCenter());
        if(distance<_circle.getRadius())   return PTIN;
        else if(distance==_circle.getRadius())   return PTON;
        else return PTOUT;
    }

    static POINT_POSITION point_and_polyline_relative(const RS_Vector& _val,const RS_Polyline& _polyLine)
    {
        auto tempA=_polyLine;
        bool isOn=false;
        bool isIn=RS_Information::isPointInsideContour(_val,&tempA,&isOn);

        if(isOn)
            return PTON;
        else if(isIn) return PTIN;
        else return PTOUT;

    }
};


#endif // PT_ALGSORT_H
