#include "test_dia.h"
#include "ui_test_dia.h"
#include <QTextBrowser>
#include <QString>
#include "rs_ellipse.h"
#include <QDebug>

class RS_Entity;
class RS_Ellipse;

test_dia::test_dia(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::test_dia)
{
    ui->setupUi(this);
}

test_dia::~test_dia()
{
    ui->show_information->clear();
    delete ui;
}


void test_dia::on_show_line_clicked()
{
    //获取主窗口
      auto appWin=QC_ApplicationWindow::getAppWindow();

    //获取主窗口文件信息
        RS_Document* d = appWin->getDocument();

    //获取文件中的entity
        if (d) {
            RS_Graphic* graphic = (RS_Graphic*)d;
            if (!graphic) {
                return;
            }

    //设置直线信息
        RS_Line* line;
//        line = new RS_Line{graphic, RS_Vector{0, 0}, RS_Vector{50, 50}};
//        line->setPen(RS_Pen(RS_Color(128, 128, 128),
//                            RS2::Width01,
//                            RS2::SolidLine));
        if(!ConnectPoint.empty())
        {
        for(it=(ConnectPoint.begin()+1);it!=ConnectPoint.end();it++)
        {
            line = new RS_Line{graphic, *(it-1), *it};
            line->setPen(RS_Pen(RS_Color(255, 0, 0),
                                        RS2::Width08,
                                       RS2::SolidLine));
            graphic->addEntity(line);
        }
 //       graphic->addEntity(line);

    //画线

        RS_GraphicView* v = appWin->getGraphicView();
        if (v) {
            v->redraw();}
        }
}
}


void test_dia::testfunction()
{
    //获取主窗口
      auto appWin=QC_ApplicationWindow::getAppWindow();

    //获取主窗口文件信息
        RS_Document* d = appWin->getDocument();

    //获取文件中的entity
        if (d) {
            RS_Graphic* graphic = (RS_Graphic*)d;
            if (!graphic) {
                return;
            }
        }
//      RS_GraphicView* v = appWin->getGraphicView();
    qDebug()<<"Process Finish";


    QList<RS_Entity *>::iterator i = d->entities.begin() ;
    QList<RS_Entity *>::iterator j = d->entities.end() -1 ;

    ui->show_information->clear();
    ConnectPoint.clear();

for(k=i;k <= j;k++)
{
    if((*k)->isVisible()){
    //直线
    switch((*k)->rtti()){
        case RS2::EntityLine :
    {
        x= (*k)->getEndpoint();
        y= (*k)->getStartpoint();

        ui->show_information->insertPlainText("直线:\t起点x:"+ QString::number(x.x) + "  y:" + QString::number(x.y) + "\t");
        ui->show_information->insertPlainText("终点:x:"+ QString::number(y.x) + "  y:" + QString::number(y.y) + "\n\n");
        break;
    }
     //圆
        case RS2::EntityCircle :
    {
        o = (*k)->getCenter();
        r = (*k)->getRadius();
        RS_Vector CirclePoint;
        CirclePoint.x = o.x-r;
        CirclePoint.y = o.y;

        ui->show_information->insertPlainText("圆:\t圆心x:"+ QString::number(o.x) + "\t\t");
        ui->show_information->insertPlainText("半径:"+ QString::number(r) + "\n\n");
        ConnectPoint.push_back(CirclePoint);
        break;
        }

        //多段线
         case RS2::EntityPolyline :
         {
        ui->show_information->insertPlainText("多段线\t");


         RS_Polyline* b=dynamic_cast<RS_Polyline*>(*k);

         if(b->isClosed())   ui->show_information->insertPlainText("(封闭)\n\n" );
         else                ui->show_information->insertPlainText("(未封闭)\n\n");

         if(b->isClosed())
         {
         RS_Vector PolylinePoint;
         PolylinePoint=b->getStartpoint();
         ConnectPoint.push_back(PolylinePoint);
         }
         break;
    }

     //椭圆
       case RS2::EntityEllipse:
    {
        RS_Ellipse* a=dynamic_cast<RS_Ellipse*>(*k);
//        center = (*k)->getCenter();
        auto vec=a->getMajorP();
        ui->show_information->insertPlainText(QString::number(vec.x));
        break;
    }

    default:
        break;




    }

}

}

}


void test_dia::on_reserve_clicked()
{
test_dia::testfunction();
}

void test_dia::on_ptn_clean_clicked()
{
    ui->show_information->clear();
}
