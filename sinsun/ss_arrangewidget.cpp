#include "ss_arrangewidget.h"
#include "ui_ss_arrangewidget.h"
#include "QMessageBox"

ss_ArrangeWidget::ss_ArrangeWidget(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ss_ArrangeWidget)
{
    ui->setupUi(this);
    l=new RS_Layer(QString("layerArrRes"));
    p=new RS_Layer(QString("layerPlank"));


}

ss_ArrangeWidget::~ss_ArrangeWidget()
{
    delete ui;
}
QList<RS_Entity*> ss_ArrangeWidget::m_finalEntityList;

void ss_ArrangeWidget::on_btn_showInfo_clicked()
{
    auto appWin=QC_ApplicationWindow::getAppWindow();
    RS_Document* d = appWin->getDocument();
    RS_Graphic* graphic = (RS_Graphic*)d;
    if(!graphic->findLayer(QString("layerArrRes")))
    {
            graphic->addLayer(l);
    }

   // m_EntityList = graphic->getEntityList();


    foreach(auto object, m_TbaEntityList)
    {
        object=NULL;
    }

    m_TbaEntityList.clear();

    foreach(auto object, m_EntityList)
    {
        if(object->getLayer()->getName()!="layerArrRes"&&object->isVisible())
            m_TbaEntityList.push_back(object);
    }





    ui->txt_arrangeNum->clear();
    for(int i=0;i<m_TbaEntityList.size();i++)
    {
        if(i==m_TbaEntityList.size()-1)
        {
            ui->txt_arrangeNum->insertPlainText("1");
        }
        else
        {
            ui->txt_arrangeNum->insertPlainText("1,");
        }

    }

}



void ss_ArrangeWidget::on_btn_arrange_clicked()
{
    auto appWin=QC_ApplicationWindow::getAppWindow();
    RS_Document* d = appWin->getDocument();
    RS_Graphic* graphic = (RS_Graphic*)d;

    QString inf= ui->txt_arrangeNum->toPlainText();
    QStringList arrangeNum= inf.split(",");
    QVector<int> arrNum;


    foreach(auto object,arrangeNum)
    {
        arrNum.push_back(object.toInt());
    }
    if(m_TbaEntityList.size()!=arrangeNum.size())
    {
        QMessageBox::critical(NULL, "critical", "输入个数与图形个数不匹配", QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
    }
    else
    {
        foreach(auto object, m_EntityArrList)
        {
            graphic->removeEntity(object);
            object=NULL;
        }
        m_EntityArrList.clear();
        graphic->update();


        for(int i=0; i<arrangeNum.size(); i++)
        {

            for(int k=0;k<arrNum[i];k++)
            {
                m_EntityArrList.push_back(m_TbaEntityList[i]->clone());
            }


        }
        plankLength=ui->txt_plankLength->toPlainText().toInt();
        plankWidth=ui->txt_plankWidth->toPlainText().toInt();

        ss_ArrangeWidget::m_finalEntityList= arrange(m_EntityArrList,plankLength,plankWidth);
        draw(m_finalEntityList);
        drawPlank(plankLength,plankWidth);


    }
}

/*
 * 排布函数
 */
QList<RS_Entity*> ss_ArrangeWidget::arrange(QList<RS_Entity*> _EntityArrList,double _plankLength, double _plankWidth)
{
    double coordinateIndexX=0,coordinateIndexY=0,subY=0;
    double maxWidth=0,maxY=0;




    for(it=_EntityArrList.begin();it!=_EntityArrList.end();it++)
    {
        double length=(*it)->getMax().x-(*it)->getMin().x,
                width=(*it)->getMax().y-(*it)->getMin().y;
        if(coordinateIndexX+length<=_plankLength&&coordinateIndexY+width<=_plankWidth)
        {
            (*it)->move(RS_Vector(coordinateIndexX-(*it)->getMin().x,coordinateIndexY-(*it)->getMin().y));
            if(width>maxWidth)  maxWidth=width;
            if(it!=(_EntityArrList.end()-1))
            {
                double lengthNext=(*(it+1))->getMax().x-(*(it+1))->getMin().x,
                        widthNext=(*(it+1))->getMax().y-(*(it+1))->getMin().y;
                subY=coordinateIndexY+width;
                while(it!=(_EntityArrList.end()-1)&&subY+widthNext-maxY<=maxWidth&&lengthNext<=length)
                {
                    it++;
                    length=(*it)->getMax().x-(*it)->getMin().x;
                    width=(*it)->getMax().y-(*it)->getMin().y;
                    if(it!=(_EntityArrList.end()-1))
                    {
                        lengthNext=(*(it+1))->getMax().x-(*(it+1))->getMin().x;
                        widthNext=(*(it+1))->getMax().y-(*(it+1))->getMin().y;
                    }
                    (*it)->move(RS_Vector(coordinateIndexX-(*it)->getMin().x,subY-(*it)->getMin().y));
                    subY+=width;



                }
            }
            coordinateIndexX+=length;
        }

        else if(coordinateIndexY+width+maxWidth<=_plankWidth)
        {
            coordinateIndexX=0;
            coordinateIndexY+=maxWidth;
            (*it)->move(RS_Vector(coordinateIndexX-(*it)->getMin().x,coordinateIndexY-(*it)->getMin().y));
            coordinateIndexX+=length;
            maxWidth=width;
            maxY+=maxWidth;
        }

    }


    return _EntityArrList;
}


/*
 *画出 _EntityArrList实体
 */
void ss_ArrangeWidget::draw(QList<RS_Entity*> _finalEntityList)
{
    auto appWin=QC_ApplicationWindow::getAppWindow();
    RS_Document* d = appWin->getDocument();
    RS_Graphic* graphic = (RS_Graphic*)d;

    foreach(auto object, _finalEntityList)
    {
        object->setLayer(l);
        graphic->addEntity(object);
    }

    RS_GraphicView* v = appWin->getGraphicView();
    if (v) {v->redraw();}

}

void ss_ArrangeWidget::drawPlank(double _plankLength, double _plankWidth)
{
    auto appWin=QC_ApplicationWindow::getAppWindow();
    RS_Document* d = appWin->getDocument();
    RS_Graphic* graphic = (RS_Graphic*)d;

    if(!graphic->findLayer(QString("layerPlank")))
    {
        graphic->addLayer(p);
    }

    foreach(auto object, graphic->getEntityList())
    {
        if(object->getLayer()==p)
        {
            graphic->removeEntity(object);
        }
    }

    RS_Polyline *plank=new RS_Polyline();
    plank->addVertex(RS_Vector(0,0));
    plank->addVertex(RS_Vector(0,_plankWidth));
    plank->addVertex(RS_Vector(_plankLength,_plankWidth));
    plank->addVertex(RS_Vector(_plankLength,0));
    plank->addVertex(RS_Vector(0,0));
    plank->setPen(RS_Pen(RS_Color(255, 0, 0),
                         RS2::Width15,
                         RS2::SolidLine));
    plank->setLayer(p);
    graphic->addEntity(plank);


}

QList<RS_Entity*> ss_ArrangeWidget::getFinalEntityList()
{
    return ss_ArrangeWidget::m_finalEntityList;
}
