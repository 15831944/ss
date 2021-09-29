#include "ncfileprocesscommon.h"

ncFileProcessCommon::ncFileProcessCommon()
{

}

QList<ncSolidData> ncFileProcessCommon::get_segments()
{
    QList<ncSolidData> result;
    for(int i=0;i<m_file->m_segment_file.size();++i)
    {
       result.push_back(m_file->m_segment_file[i]);
    }
    return result;
}

QList<RS_Entity*> ncFileProcessCommon::get_entities()
{
    QList<RS_Entity*> result;
    auto solidList=get_segments();
    for(auto it:solidList)
    {
        result.push_back(it.m_polyline);
    }
    return result;
}
