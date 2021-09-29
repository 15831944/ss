#include "ncfileio.h"
#include "ncfileprocesscommon.h"
#include "ncsolidprocesscommon.h"

std::string ncFileIo::generate_g_code()
{
    //获取数据分割结果，返回
    std::string resultGCode;
    ncFileProcessCommon commonProcesser(&m_file);
    QList<ncSolidData> cutSegments=commonProcesser.get_segments();

    //公制单位指令
    resultGCode+="G21\n";

    //绝对坐标相对坐标指令

    if(m_commad==CRELATIVE)
    {
        resultGCode+="G91\n";
    }
    else
    {
        resultGCode+="G90\n";
    }

    //刀补标记
    std::string onLaser="M07\n";
    std::string offLaser="M08\n";
    if(m_repairCommand==LEFT_TOOL_REPAIR)
    {
        onLaser="\nG41\nM07\n";
    }
    else if(m_repairCommand==RIGHT_TOOL_REPAIR)
    {
        onLaser="\nG42\nM07\n";
    }

    for(int i=0;i<cutSegments.size();++i)
    {
        //快速定位段
        resultGCode+="G00";
        RS_Vector g00EndPoint=cutSegments[i].m_g0_line->getEndpoint();
        RS_Vector g00StartPoint=cutSegments[i].m_g0_line->getStartpoint();

        if(m_commad==CRELATIVE)
        {
            std::string detaX="",detaY="";
            if(g00StartPoint.x!=g00EndPoint.x)
            {
                detaX+=(" X"+std::to_string(g00EndPoint.x-g00StartPoint.x));
            }
            if(g00StartPoint.y!=g00EndPoint.y)
            {
                detaY+=(" Y"+std::to_string(g00EndPoint.y-g00StartPoint.y));
            }

            resultGCode+=(detaX+detaY+"\n");
        }
        //绝对坐标格式
        else
        {
            resultGCode+=(" X"+std::to_string(g00EndPoint.x)+" Y"+std::to_string(g00EndPoint.y)+"\n");
        }

        //开激光(可能带刀补)
        resultGCode+=onLaser;

        RS_Vector lastRecord=g00EndPoint;

        //加工段,这里算上进退刀段一起处理
        ncSolidProcessCommon tempProcess(cutSegments[i]);
        vector<shared_ptr<RS_Entity>> keySegments=tempProcess.caculate_data_segmentation();

        for(int j=0;j<keySegments.size();++j)
        {
            if(keySegments[j]->rtti()==RS2::EntityCircle)//圆
            {
                shared_ptr<RS_Circle> tempCircle=dynamic_pointer_cast<RS_Circle>(keySegments[j]);

                if(1)//圆默认为顺时针
                {
                    resultGCode+="G02";
                }
                else
                {
                    resultGCode+="G03";
                }

                RS_Vector tempStartPoint=lastRecord;
                RS_Vector tempEndPoint=lastRecord;
                RS_Vector tempCenter=tempCircle->getCenter();
                //相对坐标格式
                if(m_commad==CRELATIVE)
                {
                    std::string detaX="",detaY="";
                    if(tempEndPoint.x!=tempStartPoint.x)
                    {
                        detaX+=(" X"+std::to_string(tempEndPoint.x-tempStartPoint.x));
                    }
                    if(tempEndPoint.y!=tempStartPoint.y)
                    {
                        detaY+=(" Y"+std::to_string(tempEndPoint.y-tempStartPoint.y));
                    }
                    resultGCode+=(detaX+detaY);
                    resultGCode+=(" I"+std::to_string(tempCenter.x-tempStartPoint.x)+" J"+std::to_string(tempCenter.y-tempStartPoint.y)+"\n");
                }
                //绝对坐标格式
                else
                {
                    resultGCode+=(" X"+std::to_string(tempEndPoint.x)+" Y"+std::to_string(tempEndPoint.y));
                    resultGCode+=(" I"+std::to_string(tempCenter.x-tempStartPoint.x)+" J"+std::to_string(tempCenter.y-tempStartPoint.y)+"\n");
                }
            }
            else if(keySegments[j]->rtti()==RS2::EntityLine)//直线
               {
                   resultGCode+="G01";
                   shared_ptr<RS_Line> tempLine=dynamic_pointer_cast<RS_Line>(keySegments[j]);
                   //相对坐标格式
                   if(m_commad==CRELATIVE)
                   {
                       RS_Vector g01StartPoint=tempLine->getStartpoint();
                       RS_Vector g01EndPoint=tempLine->getEndpoint();

                       std::string detaX="",detaY="";
                       if(g01StartPoint.x!=g01EndPoint.x)
                       {
                           detaX+=(" X"+std::to_string(g01EndPoint.x-g01StartPoint.x));
                       }
                       if(g01StartPoint.y!=g01EndPoint.y)
                       {
                           detaY+=(" Y"+std::to_string(g01EndPoint.y-g01StartPoint.y));
                       }

                       resultGCode+=(detaX+detaY+"\n");
                   }
                   //绝对坐标格式
                   else
                   {
                       RS_Vector g01EndPoint=tempLine->getEndpoint();
                       resultGCode+=(" X"+std::to_string(g01EndPoint.x)+" Y"+std::to_string(g01EndPoint.y)+"\n");
                   }

                   lastRecord=tempLine->getEndpoint();
               }
               else if(keySegments[j]->rtti()==RS2::EntityArc)//圆弧
               {
                   shared_ptr<RS_Arc> tempArc=dynamic_pointer_cast<RS_Arc>(keySegments[j]);

                   if(tempArc->isReversed())//顺时针为真
                   {
                       resultGCode+="G02";
                   }
                   else
                   {
                       resultGCode+="G03";
                   }

                   RS_Vector tempStartPoint=tempArc->getStartpoint();
                   RS_Vector tempEndPoint=tempArc->getEndpoint();
                   RS_Vector tempCenter=tempArc->getCenter();
                   //相对坐标格式
                   if(m_commad==CRELATIVE)
                   {
                       std::string detaX="",detaY="";
                       if(tempEndPoint.x!=tempStartPoint.x)
                       {
                           detaX+=(" X"+std::to_string(tempEndPoint.x-tempStartPoint.x));
                       }
                       if(tempEndPoint.y!=tempStartPoint.y)
                       {
                           detaY+=(" Y"+std::to_string(tempEndPoint.y-tempStartPoint.y));
                       }
                       resultGCode+=(detaX+detaY);
                       resultGCode+=(" I"+std::to_string(tempCenter.x-tempStartPoint.x)+" J"+std::to_string(tempCenter.y-tempStartPoint.y)+"\n");
                   }
                   //绝对坐标格式
                   else
                   {
                       resultGCode+=(" X"+std::to_string(tempEndPoint.x)+" Y"+std::to_string(tempEndPoint.y));
                       resultGCode+=(" I"+std::to_string(tempCenter.x-tempStartPoint.x)+" J"+std::to_string(tempCenter.y-tempStartPoint.y)+"\n");
                   }
                   lastRecord=tempArc->getEndpoint();
               }
            else if(keySegments[j]->rtti()==RS2::EntityPoint)
            {

            }
        }
        //关激光(可能带刀补)
        resultGCode+=offLaser;
    }

    //结束语句
    resultGCode+="M02";

    return resultGCode;
}

shared_ptr<ncFileData> ncFileIo::generate_nc_file(const vector<int>& afterSort,shared_ptr<ncFileData> afterFile)
{
    //查找顺序
    ncFileProcessCommon a(afterFile.get());
    auto tempOriEntities=a.get_entities();

    if(afterSort.size()!=tempOriEntities.size())
    {

    }

    //修改引线
    RS_Line* tempToModify=afterFile->m_segment_file[afterSort[0]].m_g0_line;
    tempToModify->setStartpoint(afterFile->m_coordinate_origin);

    for(int i=1;i<afterSort.size();++i)
    {
        tempToModify=afterFile->m_segment_file[afterSort[i]].m_g0_line;
        //引出线存在
        if(afterFile->m_segment_file[afterSort[i-1]].m_out_transition_line.m_is_exist)
        {
            auto tempOutLines=afterFile->m_segment_file[afterSort[i-1]].m_out_transition_line.m_guide_entities;
            auto tempOutPoint=tempOutLines.back()->getEndpoint();
            tempToModify->setStartpoint(tempOutPoint);
        }
        else
        {
            auto tempOutPoint=afterFile->m_segment_file[afterSort[i-1]].m_cutin_point;
            tempToModify->setStartpoint(tempOutPoint);
        }
    }

    return afterFile;
}

shared_ptr<ncFileData> ncFileIo::generate_nc_file(const QList<RS_Entity *> &inPutEntities)
{
    //最终的结果文件
    shared_ptr<ncFileData> result(new ncFileData());

    //临时数据
    result->m_coordinate_origin=RS_Vector(0,0);

//    RS_Polyline* back=new RS_Polyline(nullptr);
//    result->m_back_groud=back;

    //存储数据
    for(size_t i=0;i<inPutEntities.size();++i)
    {
        ncSolidData tempSolidData;
        tempSolidData.m_is_closed=true;
        tempSolidData.m_is_inner_membrane=false;
        tempSolidData.m_polyline=inPutEntities[i];

        ncSolidProcessCommon tempProcess=ncSolidProcessCommon(tempSolidData);
        tempSolidData=tempProcess.caculate_left_refpoints();
        result->m_segment_file.push_back(tempSolidData);
    }

    //生成g00线
    result->m_segment_file[0].m_g0_line=new RS_Line(result->m_coordinate_origin,result->m_segment_file[0].m_cutin_point);
    for(size_t i=0;i<result->m_segment_file.size()-1;++i)
    {
        RS_Vector p1=result->m_segment_file[i].m_cutin_point;
        RS_Vector p2=result->m_segment_file[i+1].m_cutin_point;
        result->m_segment_file[i+1].m_g0_line=new RS_Line(p1,p2);
    }

    return result;
}

//bool ncFileIo::read_nc_file(const QString & _filePath/*I*/,ncFile & _ncFile/*O*/)
//{
//    _ncFile.clear();
//    QFile file(_filePath);// 打开文件

//    if (file.open(QIODevice::ReadOnly))
//	{
//        QTextStream fileContent(&file);

//        double tempFeedrate(0),tempX(0),tempY(0);//临时数据存储，主要是为下面的省略参数赋值

//        ncPoint currentPoint;
//        ncLineCircleContainer tempPolyline;
//        ncMiniSegment tempSegment;
//        bool relativeCoordinatesFlag(false);

//        while(!fileContent.atEnd())
//        {
//            QByteArray tempBa = fileContent.readLine().toLatin1();
//            char* buffer=tempBa.data();

//            //现获取主轴转速和进给
//            if (strstr(buffer, "F") != NULL)  //获取进给率
//            {
//                read_num_after_str(buffer, "F", tempFeedrate);
//            }

//            //判断M操作
//            //if (strstr(buffer, "M07") != NULL)
//            if (strstr(buffer, "M08") != NULL)
//            {
//                tempSegment.set_polylines(tempPolyline);
//                tempPolyline.clear();
//                _ncFile.add_segment(tempSegment);
//                tempSegment.clear_polylines();
//            }
//            if (strstr(buffer, "G91") != NULL)//相对坐标
//            {
//                relativeCoordinatesFlag=true;
//            }

//            //进行数据的存储
//            if (strstr(buffer, "X") != NULL||strstr(buffer, "Y") != NULL||strstr(buffer, "I") != NULL||strstr(buffer, "J") != NULL)
//            {
//                //初始化临时变量
//                double x_coordinate = 0, y_coordinate = 0;

//                //临时储存上一个点
//                ncPoint prePoint=currentPoint;

//                //将结果存储起来
//                if(relativeCoordinatesFlag)
//                {
//                    strstr(buffer, "X") != NULL ? (read_num_after_str(buffer, "X", x_coordinate)) : x_coordinate = 0;
//                    strstr(buffer, "Y") != NULL ? (read_num_after_str(buffer, "Y", y_coordinate)) : y_coordinate = 0;
//                    currentPoint.set_position(prePoint.position().x+x_coordinate,prePoint.position().y+y_coordinate);
//                }
//                else
//                {
//                    //获取字符后面的数字，省略的话就使用最近的一个
//                    strstr(buffer, "X") != NULL ? (read_num_after_str(buffer, "X", x_coordinate), tempX = x_coordinate) : x_coordinate = tempX;
//                    strstr(buffer, "Y") != NULL ? (read_num_after_str(buffer, "Y", y_coordinate), tempY = y_coordinate) : y_coordinate = tempY;
//                    currentPoint.set_position(x_coordinate, y_coordinate);
//                }
//                currentPoint.set_feedrate(tempFeedrate);

//                //判断G操作
//                if (strstr(buffer, "G00") != NULL)//快速插补
//                {
//                    tempSegment.set_g0_line(ncLine(prePoint,currentPoint));
//                    continue;
//                }
//                if (strstr(buffer, "G01") != NULL)//直线插补
//                {
//                    std::shared_ptr<ncEntity> tempLine(new ncLine(prePoint,currentPoint));
//                    tempPolyline.add_line_or_circle(tempLine);
//                    continue;
//                }
//                if (strstr(buffer, "G02") != NULL||strstr(buffer, "G03") != NULL)//顺时针圆弧插补
//                {
//                    std::shared_ptr<ncArc> temp_circle(new ncArc());
//                    double radius_i(0), radius_j(0);

//                    temp_circle->set_start_point(prePoint);
//                    temp_circle->set_end_point(currentPoint);           //设置圆弧终点坐标

//                     //计算圆心并存储
//                     read_num_after_str(buffer, "I", radius_i);
//                     read_num_after_str(buffer, "J", radius_j);
//                     double ri = prePoint.position().x + radius_i, rj = prePoint.position().y + radius_j;

//                     temp_circle->set_center(vec2(ri,rj));

//                     if (strstr(buffer, "G02") != NULL)
//                     {
//                         temp_circle->set_rotation_direction(1);          //设置旋转方向
//                     }
//                     else
//                     {
//                         temp_circle->set_rotation_direction(0);
//                     }
//                     temp_circle->generate_parameter();

//                     tempPolyline.add_line_or_circle(temp_circle);        //存储
//                     continue;
//                }
//            }
//        }

//#ifdef debug_pt
//        std::ofstream debug("C:\\Users\\tongtong\\Desktop\\test.txt");
//        auto a = Nc_file.get_m_point_array();
//            for (int i = 0; i < a.size(); ++i)
//            {
//                debug << a[i].get_point().X<<" "<< a[i].get_point().Y << " " << a[i].get_point().Z << " " <<a[i].get_angle().X << " " <<a[i].get_angle().Y << " " <<a[i].get_feedrate() << " " <<a[i].get_spindl_speed() << " " <<std::endl;
//            }
//        debug.close();
//#endif

//        return true;
//	}
//	else
//	{
//        return false;

//	}

//}
