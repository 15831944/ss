#include "ss_sortwidget.h"
#include "ui_ss_sortwidget.h"
#include "ss_arrange.h"
#include "sinsun/xstest/ncfileio.h"
#include "sinsun/xstest/ptshowncfile.h"

RS_Layer* ss_SortWidget::s;
Pt_AlgSortEntities ss_SortWidget::algsort;


double getDistance(RS_Vector vec1,RS_Vector vec2)
{
    return sqrt(qPow(vec1.x-vec2.x,2)+qPow(vec1.y-vec2.y,2));
}

ss_SortWidget::ss_SortWidget(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ss_SortWidget)
{
    ui->setupUi(this);
    s=new RS_Layer(QString("空走行程"));

}


ss_SortWidget::~ss_SortWidget()
{
    delete ui;
}

QList<RS_Vector> ss_SortWidget::vectorList;

void ss_SortWidget::sort(const QList<RS_Entity *> _entityList,const int _sortMethod)
{
    auto appWin=QC_ApplicationWindow::getAppWindow();
    RS_Document* d = appWin->getDocument();
    RS_Graphic* graphic = (RS_Graphic*)d;
    RS_GraphicView* v = appWin->getGraphicView();

    vectorList.clear();
    foreach(auto object, _entityList)
    {
        switch (object->rtti())
        {
        case RS2::EntityCircle:
            vectorList.push_back(RS_Vector(object->getCenter().x+object->getRadius()*sqrt(2)/2,object->getCenter().y+object->getRadius()*sqrt(2)/2));
            break;
        default:
            vectorList.push_back(object->getStartpoint());
            break;
        }
    }

    ptShowNcFile show(graphic);
    //show.set_plank(ss_Arrange::plankLength,ss_Arrange::plankWidth);
    QList<RS_Entity*> copy_entityList=_entityList;

//    ncFileIo fileIO;

//    if(_sortMethod==1)//网格排序
//    {
//        appWin->m_result_file_list[appWin->getDocument()]=fileIO.generate_nc_file(copy_entityList);
//        //std::vector<RS_Vector> veclist=vectorList.toVector().toStdVector();
//        //algsort.set_ori_entities(copy_entityList);
//        //algsort.sort_func_grid(veclist);
//        //algsort.get_after_ncfile(appWin->m_result_file_list[appWin->getDocument()]);

//        show.m_ncFile=appWin->m_result_file_list[appWin->getDocument()];
//        show.set_plank(ss_Arrange::plankLength,ss_Arrange::plankWidth);
//        show.redraw_ncfile();
//        v->redraw();
//    }

//    if(_sortMethod==2)//局部最短空位移
//    {
//        std::vector<RS_Vector> veclist=vectorList.toVector().toStdVector();
//        algsort.set_ori_entities(copy_entityList);
//        algsort.sort_func_basic(veclist);
//        algsort.get_after_ncfile(appWin->m_result_file_list[appWin->getDocument()]);
//        //auto temEntityList=algsort.get_after_entities();

//        //appWin->m_result_file_list[appWin->getDocument()]=fileIO.generate_nc_file(order, appWin->m_result_file_list[appWin->getDocument()]);
//        show.m_ncFile=appWin->m_result_file_list[appWin->getDocument()];
//        show.set_plank(ss_Arrange::plankLength,ss_Arrange::plankWidth);
//        show.redraw_ncfile();
//        v->redraw();

//    }




}


void ss_SortWidget::draw(QList<RS_Vector> _vectorList)
{
    if(_vectorList.count()>1)
    {
        auto appWin=QC_ApplicationWindow::getAppWindow();
        RS_Document* d = appWin->getDocument();
        RS_Graphic* graphic = (RS_Graphic*)d;

        if(!graphic->findLayer(QString("空走行程")))
        {
            graphic->addLayer(s);
        }

        foreach(auto object, graphic->getEntityList())
        {
            if(object->getLayer()==s)
            {
                graphic->removeEntity(object);
            }
        }

        QList<RS_Vector>::iterator it;
        for(it=_vectorList.begin();it!=_vectorList.end()-1;it++)
        {
            RS_Line *l=new RS_Line((*it),(*(it+1)));
            l->setLayer(s);
            RS_Pen* pen=new RS_Pen();
            pen->setLineType(RS2::DashLineTiny);
            pen->setColor(RS_Color(255,255,0));
            l->setPen(*pen);
            graphic->addEntity(l);

        }
        RS_GraphicView* v = appWin->getGraphicView();
        if (v) {v->redraw();}
    }

}

void ss_SortWidget::on_pushButton_clicked()
{
    auto appWin=QC_ApplicationWindow::getAppWindow();
        if(appWin->m_result_file_list[appWin->getDocument()]!=nullptr)
        {
    qDebug()<<ss_Arrange::entitylist.count();
    if(!ss_Arrange::entitylist.isEmpty())
    {
        if(ui->radioButton->isChecked())
        {
            sort(ss_Arrange::entitylist,1);

        }
        if(ui->radioButton_2->isChecked())
        {
            sort(ss_Arrange::entitylist,2);

        }

    }
    }




}

QList<RS_Vector> ss_SortWidget::getVector()
{
    return vectorList;
}


