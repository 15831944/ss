#ifndef NCFILEPROCESSCOMMON_H
#define NCFILEPROCESSCOMMON_H

#include "ncfile.h"


class ncFileProcessCommon
{
private:
    ncFileData* m_file;

public:
    ncFileProcessCommon();

    ncFileProcessCommon(ncFileData* val){m_file=val;};

    //·µ»ØÊý¾Ý
    QList<ncSolidData> get_segments();

    QList<RS_Entity*> get_entities();
};

#endif // NCFILEPROCESSCOMMON_H
