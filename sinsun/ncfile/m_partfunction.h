#ifndef M_PARTFUNCTION_H
#define M_PARTFUNCTION_H

#include "rs_information.h"
#include "m_ncfile.h"

class M_PartFunction
{
private:
    MncSolidData m_solid;

public:
    M_PartFunction();
    M_PartFunction(MncSolidData val){m_solid=val;}

    //生成切入点,对原有切入点进行修改
    MncSolidData caculate_left_refpoints();

    //寻找长边的中点作为参考点
    MncSolidData caculate_long_middle_refpoints();

    //寻找顶点作为引入点
    MncSolidData caculate_coner_refpoints();

    //根据图形总长的比例计算引入点
    MncSolidData caculate_proportion_refpoints(double proporte);

    //根据冷却点等对数据进行分割,true为顺时针
    vector<shared_ptr<RS_Entity>> caculate_data_segmentation(bool direction=true);

    //返回所有点
    vector<RS_Vector> get_all_rsvector();
};

#endif // M_PARTFUNCTION_H
