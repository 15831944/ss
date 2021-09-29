#include "ptshowncfile.h"

ptShowNcFile::ptShowNcFile()
{

}

ptShowNcFile::ptShowNcFile(RS_Graphic* graphic)
{
    m_graphic=graphic;
}

void ptShowNcFile::prepare_layers()
{
    if (!m_graphic) return;

    //释放管理权限
    m_graphic->setOwner(false);

    //删除所有信息
    m_graphic->clear();
    m_graphic->clearBlocks();
    m_graphic->clearLayers();
    m_graphic->clearVariables();

    //图层赋值
    RS_Layer* backLayer=new RS_Layer("back");
    backLayer->setPen(RS_Pen(RS_Color(255,0,0),RS2::Width16,RS2::SolidLine));
    backLayer->lock(true);
    m_graphic->addLayer(backLayer);

    RS_Layer* solidLayer=new RS_Layer("solid");
    solidLayer->setPen(RS_Pen(RS_Color(0,255,0),RS2::Width08,RS2::SolidLine));
    m_graphic->addLayer(solidLayer);

    RS_Layer* walkLayer=new RS_Layer("walk");
    walkLayer->setPen(RS_Pen(RS_Color(248,231,28),RS2::Width08,RS2::DashLineTiny));
    walkLayer->lock(true);
    m_graphic->addLayer(walkLayer);

    RS_Layer* guideLineLayer=new RS_Layer("guide");
    guideLineLayer->setPen(RS_Pen(RS_Color(0,255,255),RS2::Width08,RS2::SolidLine));
    guideLineLayer->lock(true);
    m_graphic->addLayer(guideLineLayer);

    RS_Layer* coldPointsLayer=new RS_Layer("cold");
    coldPointsLayer->setPen(RS_Pen(RS_Color(0,0,0),RS2::Width08,RS2::SolidLine));
    coldPointsLayer->lock(true);
    m_graphic->addLayer(coldPointsLayer);

}

void ptShowNcFile::redraw_ncfile()
{
    m_graphic->clear();

    auto backLayer=m_graphic->findLayer("back");
    auto solidLayer=m_graphic->findLayer("solid");
    auto walkLayer=m_graphic->findLayer("walk");
    auto guideLineLayer=m_graphic->findLayer("guide");
    auto coldLayer=m_graphic->findLayer("cold");

    if(m_ncFile!=nullptr)
    {
        //背景显示
        if(m_ncFile->m_back_groud!=nullptr)
        {
            //添加背景
            RS_Entity* back=m_ncFile->m_back_groud;
            back->setLayer(backLayer);
            m_graphic->addEntity(back);
        }

        //添加实体和g00线
        for(size_t i=0;i<m_ncFile->m_segment_file.size();++i)
        {
            RS_Entity* tempEntity=m_ncFile->m_segment_file[i].m_polyline;
            tempEntity->setLayer(solidLayer);
            m_graphic->addEntity(tempEntity);

            RS_Line* tempLine=m_ncFile->m_segment_file[i].m_g0_line;
            tempLine->setLayer(walkLayer);
            m_graphic->addEntity(tempLine);
        }

        //修改引线部分
        for(size_t i=0;i<m_ncFile->m_segment_file.size();++i)
        {
            //重新设定
            if(m_ncFile->m_segment_file[i].m_in_transition_line.m_is_exist)
            {
                QList<RS_Entity*> all=m_ncFile->m_segment_file[i].m_in_transition_line.m_guide_entities;

                for(int i=0;i<all.size();++i)
                {
                    all[i]->setLayer(guideLineLayer);
                    m_graphic->addEntity(all[i]);
                }
            }

            if(m_ncFile->m_segment_file[i].m_out_transition_line.m_is_exist)
            {
                QList<RS_Entity*> all=m_ncFile->m_segment_file[i].m_out_transition_line.m_guide_entities;

                for(int i=0;i<all.size();++i)
                {
                    all[i]->setLayer(guideLineLayer);
                    m_graphic->addEntity(all[i]);
                }
            }

            if(m_ncFile->m_segment_file[i].m_cold_points.size()>0)
            {      
                std::set<RS_Vector> points=m_ncFile->m_segment_file[i].m_cold_points;
                for(auto it:points)
                {
                    RS_Circle* tempCircle=new RS_Circle(nullptr,RS_CircleData(it,5));
                    tempCircle->setLayer(coldLayer);
                    m_graphic->addEntity(tempCircle);
                }
            }
        }
    }
}

void ptShowNcFile::set_plank(double _length,double _width)
{
   // auto backLayer=m_graphic->findLayer("back");

    RS_Polyline *plank=new RS_Polyline();
    plank->addVertex(RS_Vector(0,0));
    plank->addVertex(RS_Vector(0,_width));
    plank->addVertex(RS_Vector(_length,_width));
    plank->addVertex(RS_Vector(_length,0));
    plank->addVertex(RS_Vector(0,0));
//    plank->setPen(RS_Pen(RS_Color(255, 0, 0),
//                         RS2::Width15,
//                         RS2::SolidLine));
    m_ncFile->m_back_groud=plank;

}


RS_Entity* ptShowNcFile::get_entity_copy(std::shared_ptr<RS_Entity> in)
{
    using namespace std;
    if(in->rtti()==RS2::EntityPolyline)
    {
        std::shared_ptr<RS_Polyline> temp=std::dynamic_pointer_cast<RS_Polyline>(in);
        auto b=PtCommonFunctions::ptcopy_polyline_deeply(temp);
        return b;
    }
    if(in->rtti()==RS2::EntityCircle)
    {
        auto temp=dynamic_pointer_cast<RS_Circle>(in);
        RS_Circle* a=new RS_Circle(nullptr,temp->getData());
        return a;
    }
}

void ptAnimationThread::run()
{
    //数据初始化
    auto appMain=QC_ApplicationWindow::getAppWindow();
    RS_GraphicView* reFresh = appMain->getGraphicView();
    RS_Document* doc=appMain->getDocument();

    graphic = (RS_Graphic*)(doc);
    if (!graphic) return;

    //添加可移动叉
    if(!isin)
    {
        isin=true;
        RS_Pen crossPen(RS_Color(255,0,0),RS2::Width20,RS2::SolidLine);

        m_cross_line1=new RS_Line(RS_Vector(-5,-5),RS_Vector(5,5));
        m_cross_line1->setPen(crossPen);
        m_cross_line2=new RS_Line(RS_Vector(-5,5),RS_Vector(5,-5));
        m_cross_line2->setPen(crossPen);
        graphic->addEntity(m_cross_line1);
        graphic->addEntity(m_cross_line2);
    }
    m_cross_line1->setVisible(true);
    m_cross_line2->setVisible(true);

    m_start_point=RS_Vector (0,0);
    RS_Line* testline1=new RS_Line(RS_Vector(0,0),RS_Vector(100,100));
    RS_Line* testline2=new RS_Line(RS_Vector(100,100),RS_Vector(50,0));
    RS_Line* testline3=new RS_Line(RS_Vector(50,0),RS_Vector(-50,0));

    m_show_line_pack.push_back(testline1);
    m_show_line_pack.push_back(testline2);
    m_show_line_pack.push_back(testline3);

    graphic->addEntity(testline1);
    graphic->addEntity(testline2);
    graphic->addEntity(testline3);

    //主循环
    while(true)
    {
        for(size_t i=0;i<m_show_line_pack.size();++i)
        {
            RS_Line* tempLine=m_show_line_pack[i];
            RS_Vector tempDirection=tempLine->getEndpoint()-tempLine->getStartpoint();
            tempDirection/=sqrt(tempDirection.squared());

//            while((tempDirection%(tempLine->getEndpoint()-m_cross_line1->getMiddlePoint()))>0)
//            {
//                if(m_stopFlag)
//                {
//                    m_cross_line1->move(m_start_point-m_cross_line1->getMiddlePoint());
//                    m_cross_line2->move(m_start_point-m_cross_line2->getMiddlePoint());
//                }
//                if(!m_time_out)
//                {
//                    m_cross_line1->move(tempDirection*0.01*m_speed);
//                    m_cross_line2->move(tempDirection*0.01*m_speed);
//                }

//                if (reFresh)  reFresh->redraw();
//                //进程控制
//                m_event.processEvents();
//            }
        }
        m_stopFlag=true;
    }
}

void ptAnimationThread::set_speed(int _speed)
{
    if(_speed>=0&&_speed<=10)
    {
        m_speed=pow(2,_speed);
    }
}

void ptAnimationThread::set_nc_file(const ncFileData& inPut)
{
    m_start_point=RS_Vector (0,0);
    RS_Line* testline1=new RS_Line(RS_Vector(0,0),RS_Vector(100,100));
    RS_Line* testline2=new RS_Line(RS_Vector(100,100),RS_Vector(50,0));
    RS_Line* testline3=new RS_Line(RS_Vector(50,0),RS_Vector(-50,0));

    m_show_line_pack.push_back(testline1);
    m_show_line_pack.push_back(testline2);
    m_show_line_pack.push_back(testline3);
}

void ptAnimationThread::clear_nc_file()
{

}

