#include "arrangement.h"
#include "ui_arrangement.h"

arrangement::arrangement(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::arrangement)
{
    ui->setupUi(this);
}

arrangement::~arrangement()
{
    delete ui;
}

void arrangement::on_arrange_clicked()
{
    auto appWin=QC_ApplicationWindow::getAppWindow();
    RS_Document* d = appWin->getDocument();
    RS_Graphic* graphic = (RS_Graphic*)d;
    EntityList = graphic->getEntityList();
 static bool once = true;
if(once)
{
    layer2 = new RS_Layer(QString("arrange"));
    graphic->addLayer(layer2);
    once=false;
}




    foreach(auto entity,EntityList)
    {
        if(entity->getLayer()==layer2)
            entity->setVisible(0);

    }
    for(it=TBA.begin();it!=TBA.end();it++)
    {
        it->polyline=NULL;
    }
    TBA.clear();
    qDebug()<<"清空TBA成功";

    for(it_polyline=PolylineList.begin();it_polyline!=PolylineList.end();it_polyline++)
    {
        (*it_polyline)=NULL;
    }
    PolylineList.clear();
    qDebug()<<"清空PolylineList成功";
    object=new QList<RS_Polyline*>;
    for(it_object=object->begin();it_object!=object->end();it_object++)
    {
        (*it_object)=NULL;
    }
    object->clear();
    qDebug()<<"清空object成功";

//      object = new QList<RS_Polyline*>;
//    TBA.clear();
//    PolylineList.clear();



//    object = new QList<RS_Polyline*>;
//    PolylineList.clear();


    object1Num=0;
    object2Num=0;
    panelLength = (ui->panelLength->toPlainText()).toDouble();
    panelWidth = (ui->panelWidth->toPlainText()).toDouble();
    object1Num= (ui->object1_num->toPlainText()).toDouble();
    object2Num= (ui->object2_num->toPlainText()).toDouble();

    RS_Vector panelPoint1 = RS_Vector(0,panelWidth);
    RS_Vector panelPoint2 = RS_Vector(panelLength,panelWidth);
    RS_Vector panelPoint3 = RS_Vector(panelLength,0);



    qDebug() <<"初始化参数";


     //画板材边框
//    auto appWin=QC_ApplicationWindow::getAppWindow();
//    RS_Document* d = appWin->getDocument();
            if(!EntityList.isEmpty())
                for(it_entity=EntityList.begin();it_entity!=EntityList.end(); it_entity++)
                {

                    if( (*it_entity)->rtti()==RS2::EntityPolyline)
                    {
                        if((*it_entity)->isVisible())
                        {
                            if((*it_entity)->getLayer()!=layer2)
                            {
                                RS_Polyline *b=dynamic_cast<RS_Polyline*>(*it_entity);
                                //if(b->isClosed())
                                PolylineList.push_back(*b);
                            }

                        }

                    }
                }
            RS_Line* line1 = new RS_Line{graphic, RS_Vector(0,0), panelPoint1};
            RS_Line* line2 = new RS_Line{graphic, panelPoint1, panelPoint2};
            RS_Line* line3 = new RS_Line{graphic, panelPoint2, panelPoint3};
            RS_Line* line4 = new RS_Line{graphic, panelPoint3, RS_Vector(0,0)};

            line1->setPen(RS_Pen(RS_Color(255, 0, 0),
                                 RS2::Width15,
                                 RS2::SolidLine));
            line2->setPen(RS_Pen(RS_Color(255, 0, 0),
                                 RS2::Width15,
                                 RS2::SolidLine));
            line3->setPen(RS_Pen(RS_Color(255, 0, 0),
                                 RS2::Width15,
                                 RS2::SolidLine));
            line4->setPen(RS_Pen(RS_Color(255, 0, 0),
                                 RS2::Width15,
                                 RS2::SolidLine));
            line1->setLayer(layer2);
            line2->setLayer(layer2);
            line3->setLayer(layer2);
            line4->setLayer(layer2);

            graphic->addEntity(line1);
            graphic->addEntity(line2);
            graphic->addEntity(line3);
            graphic->addEntity(line4);




        //初始化TBA
        if(PolylineList.isEmpty())
            return;


        it_polyline = PolylineList.begin();

        for(int j=0;j<object1Num;j++)
        {
            TBA.push_back(ToBeArranged{(it_polyline)->getMax().x-(it_polyline)->getMin().x,(it_polyline)->getMax().y-(it_polyline)->getMin().y,new RS_Polyline});
            TBA.last().polyline = dynamic_cast<RS_Polyline*>(it_polyline->clone());
            //TBA.last().polyline->setLayer(layer2);

        }



        it_polyline++;

        for(int j=0;j<object2Num;j++)
        {
            TBA.push_back(ToBeArranged{(it_polyline)->getMax().x-(it_polyline)->getMin().x,(it_polyline)->getMax().y-(it_polyline)->getMin().y,new RS_Polyline});
            TBA.last().polyline = dynamic_cast<RS_Polyline*>(it_polyline->clone());
            //TBA.last().polyline->setLayer(layer2);
        }
        qDebug() <<"初始化TBA";

        //调用排布函数
        arrange(graphic);
        qDebug() <<"排布完成";


    RS_GraphicView* v = appWin->getGraphicView();
    if (v) {
        v->redraw();}


}

/*
 *排布函数
*/
void arrangement::arrange(RS_Graphic* graphic){
    qDebug()<<"进入排序";
    double coordinateIndexX=0,coordinateIndexY=0,subY=0;
    maxWidth=0;maxY=0;int i =1;

    for(it=TBA.begin();it!=TBA.end();it++)
    {
        qDebug()<<"第" <<i<<"次循环（187）";
        if(coordinateIndexX+it->length<=panelLength&&coordinateIndexY+it->width<=panelWidth)
        {
            (*it).polyline->move(RS_Vector(coordinateIndexX-(*it).polyline->getMin().x,coordinateIndexY-(*it).polyline->getMin().y));
            object->push_back((*it).polyline);
            object->back()->setLayer(layer2);
            object->back()->setPenToActive();

        qDebug()<<"第" <<i<<"次循环（195）";

            //coordinateIndexX+=it->length;
            graphic->addEntity( object->back());
            if(it->width>maxWidth)  maxWidth=it->width;
            qDebug() <<"向右排列";
            if(it!=(TBA.end()-1))
            {
                subY=coordinateIndexY+(it)->width;
                while(it!=(TBA.end()-1)&&subY+(it+1)->width-maxY<=maxWidth&&(it+1)->length<=it->length)
                {
                    it++;
                    (*it).polyline->move(RS_Vector(coordinateIndexX-(*it).polyline->getMin().x,subY-(*it).polyline->getMin().y));
                    object->push_back((*it).polyline);
                    object->back()->setLayer(layer2);
                    object->back()->setPenToActive();
                    subY+=(it->width);
                    graphic->addEntity( object->back());
                    qDebug() <<"叠一层";
                }
                        qDebug()<<"第" <<i<<"次循环（215）";

            }
            coordinateIndexX+=it->length;
            qDebug()<<"第" <<i<<"次循环（219）";


        }

        else if(coordinateIndexY+it->width+maxWidth<=panelWidth)
        {
            coordinateIndexX=0;
            coordinateIndexY+=maxWidth;

            (*it).polyline->move(RS_Vector(coordinateIndexX-(*it).polyline->getMin().x,coordinateIndexY-(*it).polyline->getMin().y));
            object->push_back((*it).polyline);
            object->back()->setLayer(layer2);
            object->back()->setPenToActive();
            coordinateIndexX+=it->length;
            graphic->addEntity( object->back());
            maxWidth=it->width;
            maxY+=maxWidth;
            qDebug() <<"上一层";

        }
                qDebug()<<"第" <<i++<<"次循环（240）";

    }
    flag = 0;
    qDebug()<<"flag=0";


}

void arrangement::on_clear_clicked()
{


}
