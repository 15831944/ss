#include "ncfile.h"
#include "pt_algcommon.h"

//ncFile操作
class ncFileIo
{
public:
    ncFileData m_file;
    COORDINATE_COMMAND m_commad=CRELATIVE;          //相对位置
    TOOL_REPAIR m_repairCommand=NULL_TOOL_REPAIR;   //不进行刀补

public:
    //生成G代码
    std::string generate_g_code();

    //导入原始数据,该函数组合多段线形成polyline、识别非封闭图形、将各个部分组合成块QList<ncSolidData>存放入ncFileData m_file中、判断内外模
    shared_ptr<ncFileData> generate_nc_file(const vector<int>& afterSort,shared_ptr<ncFileData> afterFile);

    //    bool static read_nc_file(const QString & File_path/*I*/, ncFile & Nc_file/*O*/);
    shared_ptr<ncFileData> generate_nc_file(const QList<RS_Entity *> &inPutEntities);
};
