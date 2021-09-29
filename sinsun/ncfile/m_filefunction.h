#ifndef M_FILEFUNCTION_H
#define M_FILEFUNCTION_H

#include <QList>
#include <QListWidget>
#include <QDebug>

#include "rs_entity.h"

#include "m_ncfile.h"

class M_FileFunction
{
public:

    M_FileFunction(MncFileData* val){m_file=val;};

public:
    MncFileData* m_file;

    //图层实体排序
    QList<int> range_layer(QListWidget* layer_cut);
    QList<RS_Entity*> range_entities(QList<RS_Entity*> _inputEntity, QList<int> _layerSequence);

    //getter
    QList<MncSolidData> get_segments();
    QList<RS_Entity*> get_entities();

};

#endif // M_FILEFUNCTION_H
