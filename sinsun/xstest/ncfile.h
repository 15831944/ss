#ifndef NCFILE_H
#define NCFILE_H

#include <vector>
#include <set>
#include "rs_block.h"
#include "rs_graphic.h"
#include "rs_arc.h"
#include "rs_circle.h"
#include "rs_insert.h"
#include "rs_math.h"
#include "rs_ellipse.h"
#include "rs_polyline.h"
#include "rs_line.h"
#include "rs_point.h"
#include "rs_vector.h"


enum COORDINATE_COMMAND {CABSOLUTE,CRELATIVE};
enum TOOL_REPAIR {NULL_TOOL_REPAIR,LEFT_TOOL_REPAIR,RIGHT_TOOL_REPAIR};


struct guideLineData
{
    bool m_is_exist=false;//是否存在
    QList<RS_Entity*> m_guide_entities;//引入引出加工段，引入段第一个实体起点是空走行程的终点，引出段最后一个实体终点是空走行程终点
};


//加工实体
struct ncSolidData
{
    //排布前
    RS_Entity* m_polyline;//加工块
    bool m_is_closed;//是否为封闭图形
    bool m_is_inner_membrane;//是否为内膜图形,true是内膜

    //排序后
    RS_Vector m_cutin_point;//切入点
    RS_Line* m_g0_line;//g00线

    //工艺
    guideLineData m_in_transition_line;//切入线
    guideLineData m_out_transition_line;//切出线
    std::set<RS_Vector> m_cold_points;//冷却点
};

//ncFile数据
struct ncFileData
{
    QList<ncSolidData> m_segment_file;//加工段,加工段包含加工块QList<ncSolidData>，加工块包含加工实体ncSolidData
    RS_Vector m_coordinate_origin;//坐标原点
    RS_Polyline* m_back_groud;//板件
};





#endif // NCFILE_H
