#include "ss_arrange.h"
#include "ui_ss_arrange.h"
#include "sinsun/xstest/ncfileio.h"
#include "sinsun/xstest/ptshowncfile.h"

QList<RS_Entity *> ss_Arrange::entitylist;
double ss_Arrange:: plankLength=0;
double ss_Arrange:: plankWidth=0;

ss_Arrange::ss_Arrange(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ss_Arrange)
{
    ui->setupUi(this);
}

ss_Arrange::~ss_Arrange()
{
    delete ui;
}

QList<RS_Entity *> ss_Arrange::arrange(QList<Part*> _partArrList, double _plankLength, double _plankWidth)
{
    QList<RS_Entity*> arrange_final;
    //    return test;
    double coordinateIndexX=0,coordinateIndexY=0,subY=0;
    double maxWidth=0,maxY=0;



    QList<Part*>::iterator it;
    for(it=_partArrList.begin();it!=_partArrList.end();it++)
    {
        double length=(*it)->maxW;
        double width=(*it)->maxH;
        if(coordinateIndexX+length<=_plankLength&&coordinateIndexY+width<=_plankWidth)
        {
            foreach(auto item,(*it)->part)
            {
                auto entity=item->clone();
                entity->move(RS_Vector(coordinateIndexX-(*it)->getMin().x,coordinateIndexY-(*it)->getMin().y));
                //entity->setLayer(l);
                arrange_final.push_back(entity);
            }
            if(width>maxWidth)  maxWidth=width;
            if(it!=(_partArrList.end()-1))
            {
                double lengthNext=(*(it+1))->getMax().x-(*(it+1))->getMin().x;
                double widthNext=(*(it+1))->getMax().y-(*(it+1))->getMin().y;
                subY=coordinateIndexY+width;
                while(it!=(_partArrList.end()-1) && subY+widthNext-maxY<=maxWidth&&lengthNext<=length)
                {
                    it++;
                    length=(*it)->getMax().x-(*it)->getMin().x;
                    width=(*it)->getMax().y-(*it)->getMin().y;
                    if(it!=(_partArrList.end()-1))
                    {
                        lengthNext=(*(it+1))->getMax().x-(*(it+1))->getMin().x;
                        widthNext=(*(it+1))->getMax().y-(*(it+1))->getMin().y;
                    }
                    foreach(auto item,(*it)->part)
                    {
                        auto entity=item->clone();
                        entity->move(RS_Vector(coordinateIndexX-(*it)->getMin().x,subY-(*it)->getMin().y));
                        //entity->setLayer(l);
                        arrange_final.push_back(entity);
                    }
                    subY+=width;
                }
            }
            coordinateIndexX+=length;

        }
        else if(coordinateIndexY+width+maxWidth<=_plankWidth)
        {
            coordinateIndexX=0;
            coordinateIndexY+=maxWidth;
            foreach(auto item,(*it)->part)
            {
                auto entity=item->clone();
                entity->move(RS_Vector(coordinateIndexX-(*it)->getMin().x,coordinateIndexY-(*it)->getMin().y));
                //entity->setLayer(l);
                arrange_final.push_back(entity);
            }
            coordinateIndexX+=length;
            maxWidth=width;
            maxY+=maxWidth;
        }

    }
    foreach(auto item,arrange_final)
    {
        item->setSelected(0);
    }


    return arrange_final;

}

void ss_Arrange::draw(QList<RS_Entity*> _finalEntityList)
{
    auto appWin=QC_ApplicationWindow::getAppWindow();
    RS_Document* d = appWin->getDocument();
    RS_Graphic* graphic = (RS_Graphic*)d;

    foreach(auto object, _finalEntityList)
    {
        //object->setLayer(l);
        graphic->addEntity(object);
    }

    RS_GraphicView* v = appWin->getGraphicView();
    if (v) {v->redraw();}
}

void ss_Arrange::drawPlank(double _plankLength, double _plankWidth)
{

    auto appWin=QC_ApplicationWindow::getAppWindow();
    RS_Document* d = appWin->getDocument();
    RS_Graphic* graphic = (RS_Graphic*)d;

    //RS_Layer* l1=new RS_Layer(QString("板材"));
    //l1->lock(1);
//    if(!graphic->findLayer(QString("板材")))
//    {
//        graphic->addLayer(l1);

//    }

    RS_Polyline *plank=new RS_Polyline();
    plank->addVertex(RS_Vector(0,0));
    plank->addVertex(RS_Vector(0,_plankWidth));
    plank->addVertex(RS_Vector(_plankLength,_plankWidth));
    plank->addVertex(RS_Vector(_plankLength,0));
    plank->addVertex(RS_Vector(0,0));
    plank->setPen(RS_Pen(RS_Color(255, 0, 0),
                         RS2::Width15,
                         RS2::SolidLine));
    //plank->setLayer(l1);
    graphic->addEntity(plank);

}

void ss_Arrange::on_pushButton_2_clicked()
{
    //l=new RS_Layer(QString("零件"));
    QList<Part*> partlist;
    //获取零件库待排布零件信息
    int row=0;
    if(ss_PartFrame::partList->count())
    {
        while(row<ss_PartFrame::partList->count())
        {
            QListWidgetItem *item=ss_PartFrame::partList->item(row);
            PartPreview* widget=(PartPreview*)ss_PartFrame::partList->itemWidget(item);
            if(widget->checkbox->isChecked() || ui->arrange_all->isChecked())
            {
                partlist.push_back(&widget->m_part);
            }

            row++;
        }
        if(!partlist.isEmpty())
        {
            auto appWin=QC_ApplicationWindow::getAppWindow();
            appWin->createNewDocument("排布结果");
            RS_Document* d = appWin->getDocument();
            d->setFilename("排布结果");
            RS_Graphic* graphic = (RS_Graphic*)d;
            QList<Part*> partlist_num;

            foreach(auto item,partlist)
            {
                for(int i=0;i<item->partNum;i++)
                {
                    partlist_num.push_back(item->clone());
                }
            }

            plankLength=ui->plankLength->text().toDouble();
            plankWidth=ui->plankWidth->text().toDouble();



            //排布
            entitylist=arrange(partlist_num,plankLength,plankWidth);
            //默认网格排序
            QList<RS_Entity*> copy;
            foreach(auto item, entitylist)
            {
                copy.push_back(item->clone());
            }

//             ncFileIo fileIO;
//             //appWin->m_result_file_list[appWin->getDocument()]=fileIO.generate_nc_file(copy);

//             //显示部分
//             ptShowNcFile show(graphic);
//             show.m_ncFile=appWin->m_result_file_list[appWin->getDocument()];
//             show.prepare_layers();
//             show.set_plank(plankLength,plankWidth);
//             show.redraw_ncfile();

//             //刷新
//             RS_GraphicView* v = appWin->getGraphicView();
//             if (v) {v->redraw();}
        }

    }



}

void ss_Arrange::on_pushButton_clicked()
{
    close();
}
