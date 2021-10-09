#include "m_ncfile.h"
#include "m_partfunction.h"
#include "m_filefunction.h"

MncFileData MncFileIo::generate_ncFile(const QList<RS_Entity *> &inPutEntities)
{
    //初始化m_file
    m_file.m_segment_file.clear();
    m_file.m_coordinate_origin=RS_Vector(0,0);

    RS_Polyline* back=new RS_Polyline(nullptr);
    m_file.m_back_groud=back;

    //存储数据
    M_FileFunction fileFunction(&m_file);
    //图层排序
    QList<int> layer_range = fileFunction.range_layer(m_layerCut);
    //实体图层
    QList<RS_Entity*> entities_ranged = fileFunction.range_entities(inPutEntities, layer_range);

    foreach(auto item_entity, entities_ranged)
    {
        MncSolidData tempSolidData;
        tempSolidData.m_is_closed=true;
        tempSolidData.m_is_inner=false;
        tempSolidData.m_part=item_entity;

        M_PartFunction tem_partFunction = M_PartFunction(tempSolidData);
        //计算切入点
        tempSolidData=tem_partFunction.caculate_left_refpoints();
        m_file.m_segment_file.push_back(tempSolidData);
    }
    return m_file;
}

string MncFileIo::generate_gcode()
{
    MGUnit gUnit;
    //获取数据分割结果，返回
    std::string resultGCode;
    M_FileFunction commonProcesser(&m_file);
    QList<MncSolidData> cutSegments=commonProcesser.get_segments();

    //绝对坐标相对坐标指令
    if(m_commad==MRELATIVE)
    {
        resultGCode+=gUnit.generate_G(G91);
    }
    else
    {
        resultGCode+=gUnit.generate_G(G90);
    }

    for(int i=0;i<cutSegments.size();++i)
    {
        int layer = cutSegments[i].m_part->m_nc_information.layer_cut;
        //快速定位段
        RS_Vector g00StartPoint, g00EndPoint;
        if(!i)
        {
            g00StartPoint = m_file.m_coordinate_origin;
            g00EndPoint = cutSegments[i].getCutInPoint();
        }
        else
        {
            g00StartPoint = cutSegments[i-1].getCutOutPoint();
            g00EndPoint = cutSegments[i].getCutInPoint();
        }
        //相对坐标
        if(m_commad==MRELATIVE)
        {
            std::string deta = "";
            if(g00StartPoint.x!=g00EndPoint.x && g00StartPoint.y!=g00EndPoint.y)
            {
                std::string detaX = ("X"+std::to_string(g00EndPoint.x-g00StartPoint.x));
                std::string detaY = ("Y"+std::to_string(g00EndPoint.y-g00StartPoint.y));
                deta = detaX + " " + detaY;
            }
            else if(g00StartPoint.x!=g00EndPoint.x)
            {
               std::string detaX = ("X"+std::to_string(g00EndPoint.x-g00StartPoint.x));
               deta = detaX;
            }
            else
            {
                std::string detaY = ("Y"+std::to_string(g00EndPoint.y-g00StartPoint.y));
                deta = detaY;
            }

            resultGCode+=gUnit.generate_G(G00, deta);
        }
        //绝对坐标格式
        else
        {
            std::string coord ="X"+std::to_string(g00EndPoint.x)+" Y"+std::to_string(g00EndPoint.y);
            resultGCode+=gUnit.generate_G(G00, coord);
        }

        //开激光，标记图层信息
        resultGCode+=gUnit.generate_M(M07, layer);

        RS_Vector lastRecord=g00EndPoint;

        //加工段,这里算上进退刀段一起处理
        M_PartFunction tempProcess(cutSegments[i]);
        vector<shared_ptr<RS_Entity>> keySegments=tempProcess.caculate_data_segmentation();

        for(int j=0;j<keySegments.size();++j)
        {
            if(keySegments[j]->rtti()==RS2::EntityCircle)//圆
            {
                std::string tem_line="";
                shared_ptr<RS_Circle> tempCircle=dynamic_pointer_cast<RS_Circle>(keySegments[j]);

                if(1)//圆默认为顺时针
                {
                    //resultGCode+="G02";
                }
                else
                {
                    //resultGCode+="G03";
                }

                RS_Vector tempStartPoint=lastRecord;
                RS_Vector tempEndPoint=lastRecord;
                RS_Vector tempCenter=tempCircle->getCenter();
                std::string _coord="";
                //相对坐标格式
                   if(m_commad==MRELATIVE)
                {
                    _coord+=("I"+std::to_string(tempCenter.x-tempStartPoint.x)
                             +" J"+std::to_string(tempCenter.y-tempStartPoint.y));
                }
                //绝对坐标格式
                else
                {
                    _coord= "X"+std::to_string(tempEndPoint.x)
                               +" Y"+std::to_string(tempEndPoint.y)
                               +" I"+std::to_string(tempCenter.x-tempStartPoint.x)
                               +" J"+std::to_string(tempCenter.y-tempStartPoint.y);
                }
                resultGCode+=gUnit.generate_G(G02, _coord);
            }
            else if(keySegments[j]->rtti()==RS2::EntityLine)//直线
               {
                   //resultGCode+="G01";
                   shared_ptr<RS_Line> tempLine=dynamic_pointer_cast<RS_Line>(keySegments[j]);
                   //相对坐标格式
                   if(m_commad==MRELATIVE)
                   {
                       RS_Vector g01StartPoint=tempLine->getStartpoint();
                       RS_Vector g01EndPoint=tempLine->getEndpoint();
                       std::string deta="";
                       if(g01StartPoint.x!=g01EndPoint.x && g01StartPoint.y!=g01EndPoint.y)
                       {
                           std::string detaX = "X"+std::to_string(g01EndPoint.x-g01StartPoint.x);
                           std::string detaY = "Y"+std::to_string(g01EndPoint.y-g01StartPoint.y);
                           deta = detaX + " " + detaY;
                       }
                       else if(g01StartPoint.x!=g01EndPoint.x)
                       {
                           std::string detaX = "X"+std::to_string(g01EndPoint.x-g01StartPoint.x);
                           deta = detaX;
                       }
                       else
                       {
                           std::string detaY = "Y"+std::to_string(g01EndPoint.y-g01StartPoint.y);
                           deta = detaY;
                       }

                       resultGCode+=gUnit.generate_G(G01, deta);
                   }
                   //绝对坐标格式
                   else
                   {
                       RS_Vector g01EndPoint=tempLine->getEndpoint();
                       std::string _coord = "X"+std::to_string(g01EndPoint.x)+" Y"+std::to_string(g01EndPoint.y);
                       resultGCode+=gUnit.generate_G(G01, _coord);
                   }

                   lastRecord=tempLine->getEndpoint();
               }
               else if(keySegments[j]->rtti()==RS2::EntityArc)//圆弧
               {
                   shared_ptr<RS_Arc> tempArc=dynamic_pointer_cast<RS_Arc>(keySegments[j]);
                   G_OPERATE _operate;

                   if(tempArc->isReversed())//顺时针为真
                   {
                       _operate = G02;
                   }
                   else
                   {

                       _operate = G03;
                   }

                   RS_Vector tempStartPoint=tempArc->getStartpoint();
                   RS_Vector tempEndPoint=tempArc->getEndpoint();
                   RS_Vector tempCenter=tempArc->getCenter();
                   //相对坐标格式
                   if(m_commad==MRELATIVE)
                   {
                       std::string deta="";
                       if(tempEndPoint.x!=tempStartPoint.x && tempEndPoint.y!=tempStartPoint.y)
                       {
                           std::string detaX = "X"+std::to_string(tempEndPoint.x-tempStartPoint.x);
                           std::string detaY = "Y"+std::to_string(tempEndPoint.y-tempStartPoint.y);
                           deta = detaX + " " + detaY;
                       }
                       else if(tempEndPoint.x!=tempStartPoint.x)
                       {
                           std::string detaX = "X"+std::to_string(tempEndPoint.x-tempStartPoint.x);
                           deta = detaX;
                       }
                       else
                       {
                           std::string detaY = "Y"+std::to_string(tempEndPoint.y-tempStartPoint.y);
                           deta = detaY;
                       }
                       deta += " I"+std::to_string(tempCenter.x-tempStartPoint.x)+" J"+std::to_string(tempCenter.y-tempStartPoint.y);
                       resultGCode+=gUnit.generate_G(_operate, deta);
                   }
                   //绝对坐标格式
                   else
                   {
                       std::string _coord = "";
                       _coord = "X"+std::to_string(tempEndPoint.x)
                               +" Y"+std::to_string(tempEndPoint.y)
                               +" I"+std::to_string(tempCenter.x-tempStartPoint.x)
                               +" J"+std::to_string(tempCenter.y-tempStartPoint.y);
                       resultGCode+=gUnit.generate_G(_operate, _coord);
                   }
                   lastRecord=tempArc->getEndpoint();
               }
            else if(keySegments[j]->rtti()==RS2::EntityPoint)
            {

            }
        }
        //关激光
        resultGCode+=gUnit.generate_M(M08);
    }

    return resultGCode;
}

void MncFileIo::setLayerCut(QListWidget * layerCut)
{
    m_layerCut = layerCut;
}


std::string MGUnit::generate_lineNum()
{
    std::string result = "N";
    std::stringstream test;
    std::string str_line = std::to_string(m_lineNum);
    m_lineNum = m_lineNum + 10;
    test<< setw(3)<< setfill('0')<< str_line;
    result += test.str();
    return result;
}

string MGUnit::generate_M(M_OPERATE _operate, int _layer)
{
    std::string result;
    std::string line = generate_lineNum();
    result = result + line;
    switch (_operate) {
    case M07:
    {
        result = result + " M07";
        if(_layer>=0)
        {
            std::string layer = " K" + std::to_string(_layer);
            result = result + layer;
        }
        m_layer = _layer;
        break;
    }
    case M08:
    {
        result = result + " M08";
        m_layer = -1;
        break;
    }

    }
    result += "\n";
    return result;
}

string MGUnit::generate_G(G_OPERATE _operate, string _coord)
{
    std::string result;
    std::string line = generate_lineNum();
    result = result + line;
    switch (_operate) {
    case G00:
    {
        result = result + " G00" + " " + _coord;
        break;
    }
    case G01:
    {
        result = result + " G01" + " " + _coord
                + " F$LINE_VEL_" + std::to_string(m_layer) + "$"
                + " E$LINE_ACC_" + std::to_string(m_layer) + "$"
                + " E$LINE_DEC_" + std::to_string(m_layer) + "$";
        break;
    }
    case G02:
    {
        result = result + " G02" + " " + _coord
                + " F$CIRC_VEL_" + std::to_string(m_layer) + "$"
                + " E$CIRC_ACC_" + std::to_string(m_layer) + "$"
                + " E$CIRC_DEC_" + std::to_string(m_layer) + "$";
        break;
    }
    case G03:
    {
        result = result + " G03" + " " + _coord
                + " F$CIRC_VEL_" + std::to_string(m_layer) + "$"
                + " E$CIRC_ACC_" + std::to_string(m_layer) + "$"
                + " E$CIRC_DEC_" + std::to_string(m_layer) + "$";
        break;
    }
    case G90:
    {
        result = result + " G90";
        break;
    }
    case G91:
    {
        result = result + " G91";
        break;
    }
    }
    result += "\n";
    return result;
}
