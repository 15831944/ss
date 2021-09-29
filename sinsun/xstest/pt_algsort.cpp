#include "pt_algsort.h"
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
#include "rs_spline.h"
#include "lc_splinepoints.h"
#include "ncfileprocesscommon.h"

void Pt_AlgSortEntities::sort_func_basic(std::vector<RS_Vector>& _ConnectPoint)
{
    m_after_sort_sequence.clear();
    double minRef=__DBL_MAX__;
    int startIndex=0;
    for(int i=0;i<_ConnectPoint.size();++i)
    {
        double tempDistance=_ConnectPoint[i].distanceTo(RS_Vector(0,0));
        if(tempDistance<minRef)
        {
            minRef=tempDistance;
            startIndex=i;
        }
    }

    tsp test(_ConnectPoint);              // read in command line input
    test.nearest_neighbor_basic(startIndex);        // run nearest neighbor function

    std::vector<RS_Vector> tempPointArray;
    for(size_t i=0;i<_ConnectPoint.size();++i)
    {
        tempPointArray.push_back(_ConnectPoint[test.get_solution_num(i)]);
        m_afterEntities.push_back(m_oriEntities[test.get_solution_num(i)]);
        m_after_sort_sequence.push_back(test.get_solution_num(i));
    }

    _ConnectPoint=tempPointArray;
}

void Pt_AlgSortEntities::sort_func_grid(std::vector<RS_Vector> &_ConnectPoint)
{
    m_after_sort_sequence.clear();

    set<RS_Vector> result;
    for(auto it:_ConnectPoint)
    {
        result.insert(it);
    }

    for(auto it:result)
    {
        for(int i=0;i<_ConnectPoint.size();++i)
        {
            if(_ConnectPoint[i]==it)
            {
                m_after_sort_sequence.push_back(i);
            }
        }
    }
}

QList<RS_Vector> Pt_AlgSortEntities::get_after_points()
{

}


Pt_AlgSort::Pt_AlgSort(){}

Pt_AlgSort::~Pt_AlgSort(){}

void Pt_AlgSort::get_after_ncfile(shared_ptr<ncFileData> afterFile)
{
    //查找顺序
    ncFileProcessCommon a(afterFile.get());
    auto tempOriEntities=a.get_entities();

    if(m_after_sort_sequence.size()!=tempOriEntities.size())
    {
        return;
    }

    //修改引线
    RS_Line* tempToModify=afterFile->m_segment_file[m_after_sort_sequence[0]].m_g0_line;
    tempToModify->setStartpoint(afterFile->m_coordinate_origin);

    for(int i=1;i<m_after_sort_sequence.size();++i)
    {
        tempToModify=afterFile->m_segment_file[m_after_sort_sequence[i]].m_g0_line;
        //引出线存在
        if(afterFile->m_segment_file[m_after_sort_sequence[i-1]].m_out_transition_line.m_is_exist)
        {
            auto tempOutLines=afterFile->m_segment_file[m_after_sort_sequence[i-1]].m_out_transition_line.m_guide_entities;
            auto tempOutPoint=tempOutLines.back()->getEndpoint();
            tempToModify->setStartpoint(tempOutPoint);
        }
        else
        {
            auto tempOutPoint=afterFile->m_segment_file[m_after_sort_sequence[i-1]].m_cutin_point;
            tempToModify->setStartpoint(tempOutPoint);
        }
    }
}


