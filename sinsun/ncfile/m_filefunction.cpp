#include "m_filefunction.h"
#include "sinsun/xsui/ss_layercutitem.h"

QList<int> M_FileFunction::range_layer(QListWidget *layer_cut)
{
    QList<int> result;
    for(int i=1; i<=layer_cut->count(); i++)
    {
        for(int j=0; j<layer_cut->count(); j++)
        {
            QVariant tem_var = layer_cut->item(j)->data(Qt::UserRole);
            ss_LayerCutItem* tem_layerItem = tem_var.value<ss_LayerCutItem*>();
            if(tem_layerItem->getSequence() == i)
            {
                result.push_back(j);
            }
        }
    }
    return result;
}

QList<RS_Entity *> M_FileFunction::range_entities(QList<RS_Entity *> _inputEntity, QList<int> _layerSequence)
{
    QList<RS_Entity *> result;
    foreach(int index, _layerSequence)
    {
        foreach(auto entity, _inputEntity)
        {
            if(entity->m_nc_information.layer_cut == index)
            {
                result.push_back(entity);
            }
        }
    }
    return result;
}

QList<MncSolidData> M_FileFunction::get_segments()
{
    QList<MncSolidData> result;
    for(int i=0;i<m_file->m_segment_file.size();++i)
    {
       result.push_back(m_file->m_segment_file[i]);
    }
    return result;
}

QList<RS_Entity *> M_FileFunction::get_entities()
{
    QList<RS_Entity*> result;
    auto solidList=get_segments();
    for(auto it:solidList)
    {
        result.push_back(it.m_part);
    }
    return result;
}
