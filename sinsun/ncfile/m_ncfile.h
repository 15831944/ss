#ifndef M_NCFILE_H
#define M_NCFILE_H

#include <QObject>
#include <vector>
#include <set>
#include <QListWidget>
#include <iomanip>
#include <QString>
#include <sstream>

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

#include "sinsun/xsui/ss_layercutitem.h"

/**
 * @brief M_NcFile::M_NcFile
 *
 */

enum MCOORDINATE_COMMAND {MABSOLUTE,MRELATIVE};
enum MTOOL_REPAIR {MNULL_TOOL_REPAIR,MLEFT_TOOL_REPAIR,MRIGHT_TOOL_REPAIR};//暂时不用刀补
enum M_OPERATE {M07, M08};
enum G_OPERATE {G00, G01, G02, G03, G90, G91};


using namespace std;

struct MguideLineData
{
    bool m_is_exist=false;//是否存在
    QList<RS_Entity*> m_guide_entities;//引入引出加工段，引入段第一个实体起点是空走行程的终点，引出段最后一个实体终点是空走行程终点
};


//加工实体
struct MncSolidData
{
    RS_Entity* m_part;//加工块
    bool m_is_closed = true ;//是否为封闭图形，默认封闭
    bool m_is_inner = false;//是否为内膜图形,默认外膜

    RS_Vector m_cutin_point_inPart;//切入点
    RS_Vector getCutInPoint(){
        if(m_in_transition_line.m_is_exist)
        {
            return m_in_transition_line.m_guide_entities.first()->getStartpoint();
        }
        else
        {
            return m_cutin_point_inPart;
        }

    }
    RS_Vector getCutOutPoint(){
        if(m_in_transition_line.m_is_exist)
        {
            return m_out_transition_line.m_guide_entities.last()->getEndpoint();
        }
        else
        {
            return m_cutin_point_inPart;
        }

    }

    //工艺
    MguideLineData m_in_transition_line;//切入线
    MguideLineData m_out_transition_line;//切出线
    std::set<RS_Vector> m_cold_points;//冷却点
};

//ncFile数据
struct MncFileData
{
    QList<MncSolidData> m_segment_file;//加工段,包含加工块
    RS_Vector m_coordinate_origin;//坐标原点
    RS_Polyline* m_back_groud;//板件
};


class MncFileIo
{
public:
    QListWidget* m_layerCut;                            //图层信息
    MncFileData m_file;
    MCOORDINATE_COMMAND m_commad=MRELATIVE;             //相对位置
    MTOOL_REPAIR m_repairCommand=MNULL_TOOL_REPAIR;     //不进行刀补

    //初始化函数
    void setLayerCut(QListWidget*);

    //设置MncFileData
    void setData(MncFileData _file){
        m_file = _file;
    }

    //将原始数据排序、生成引线、计算空位移行程
    MncFileData generate_ncFile(const QList<RS_Entity *> &inPutEntities);
    //生成G代码
    std::string generate_gcode();
};

class MGUnit
{
private:
    int m_lineNum = 0;

    std::string generate_lineNum();
    int m_layer = -1;
public:
    std::string generate_M(M_OPERATE _operate, int layer = -1);//生成M07,M08指令
    std::string generate_G(G_OPERATE _operate, std::string _coord = "");

};



#endif // M_NCFILE_H
