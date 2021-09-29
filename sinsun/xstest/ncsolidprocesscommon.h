#ifndef NCSOLID_H
#define NCSOLID_H

#include <vector>
#include <memory>
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
#include "ncfile.h"

using std::vector;
using std::set;
using std::shared_ptr;
using std::make_shared;

class ncSolidProcessCommon
{
private:
    ncSolidData m_solid;
public:
    ncSolidProcessCommon();
    ncSolidProcessCommon(ncSolidData val){m_solid=val;}

    //生成切入点,对原有切入点进行修改
    ncSolidData caculate_left_refpoints();

    //寻找长边的中点作为参考点
    ncSolidData caculate_long_middle_refpoints();

    //寻找顶点作为引入点
    ncSolidData caculate_coner_refpoints();

    //根据图形总长的比例计算引入点
    ncSolidData caculate_proportion_refpoints(double proporte);

    //根据冷却点等对数据进行分割,true为顺时针
    vector<shared_ptr<RS_Entity>> caculate_data_segmentation(bool direction=true);

    //返回所有点
    vector<RS_Vector> get_all_rsvector();
};

#endif // NCSOLID_H
