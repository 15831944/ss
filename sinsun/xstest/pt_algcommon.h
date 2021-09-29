#ifndef PT_ALGCOMMON_H
#define PT_ALGCOMMON_H

#include "rs_document.h"
#include <vector>
#include "QList"
#include "city.h"
#include "algorithms.h"
#include "ncfile.h"

namespace PtCommonFunctions
{
    //将map转为QList（丢掉键）
    template <typename T1,typename T2>
    QList<T2> drop_keys(map<T1,T2> val)
    {
        QList<T2> result;
        for(auto it:val)
        {
            result.push_back(it.second);
        }
        return result;
    }

    //读取字符后面的数字
    bool read_num_after_str(const char* buff/*I*/, const char* target/*I*/, double & ans/*O*/);

    //polyline类深拷贝
    shared_ptr<RS_Polyline> ptcopy_polyline_deeply(RS_Polyline* in);

    RS_Polyline* ptcopy_polyline_deeply(shared_ptr<RS_Polyline> in);

    //过滤函数，过滤掉未被选择的实体
    map<int,RS_Entity*> get_selected_entities(const QList<RS_Entity *> val);
}

#endif // PT_ALGCOMMON_H
