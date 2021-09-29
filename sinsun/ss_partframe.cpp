#include "ss_partframe.h"
#include "ui_ss_partframe.h"
#include "partpreview.h"

QList<Part> ss_PartFrame:: m_partList;
QList<WidgetItem> ss_PartFrame::m_widgetList;
QListWidget* ss_PartFrame::partList;

ss_PartFrame::ss_PartFrame(QWidget *parent) :
    QFrame(parent),
    ui(new Ui::ss_PartFrame)
{
    ui->setupUi(this);
    partList=ui->partList;
    setWindowFlag(Qt::FramelessWindowHint);
    //partList->setFocusPolicy(Qt::NoFocus);
    button = ui->toolButton;

    QWidget *Scheduledboard = new QWidget();   //已排零件库
    ui->tabWidget->addTab(Scheduledboard,tr("已排板材库"));


     ui->toolButton->setIcon(QIcon(QPixmap(":/extui/close.png")));
     ui->toolButton->setStyleSheet("QToolButton{border-radius:4px; background-color: transparent;}"
                                "QToolButton:hover{border-radius:4px; border: 1px solid QColor(200, 200, 200); background-color: transparent;}");


    ui->btn_selectAll->setIcon(QIcon(":/icons/全选.svg"));
    ui->btn_selectNon->setIcon(QIcon(":/icons/未全选.svg"));
    ui->btn_selectRev->setIcon(QIcon(":/icons/反选.svg"));
    ui->btn_delete->setIcon(QIcon(":/icons/清除.svg"));

    connect(ui->toolButton,SIGNAL(clicked()),this,SLOT(slotClose()));


//action->setIcon(QIcon(":/icons/全选.svg"));

}

void ss_PartFrame::slotClose(){


    this->close();
}


QToolButton* ss_PartFrame::returnButton(){
    return ui->toolButton;
}


ss_PartFrame::~ss_PartFrame()
{
    delete ui;
}

QList<Part> ss_PartFrame::spiltPart(QList<RS_Entity *> _entitylist)
{

    QList<Part> partList;
    //分割零件
    //    QList<RS_Entity *>::iterator it;
    //    for(it=_entitylist.begin();it<_entitylist.end();it=it+2)
    //    {
    //        Part part;
    //        part.part.push_back((*it)->clone());
    //        part.part.push_back((*(it+1))->clone());
    //        part.partNum=1;
    //        part.originalEntity.push_back(*it);
    //        part.originalEntity.push_back(*(it+1));
    //        foreach(auto item,part.part)
    //        {
    //            if(item->getMax().x-item->getMin().x>part.maxW)
    //            {
    //                part.maxW=item->getMax().x-item->getMin().x;
    //            }

    //            if(item->getMax().y-item->getMin().y>part.maxH)
    //            {
    //                part.maxH=item->getMax().y-item->getMin().y;
    //            }

    //        }
    //        partList.push_back(part);
    //    }

//    QList<RS_Entity *> entityList=_entitylist;
//    foreach(auto item_entity, entityList)
//    {
//        RS_Entity *inside = getInsider(item_entity,entityList);
//        if(inside!=NULL)
//        {
//            entityList.removeOne(inside);
//        }
//        entityList.removeOne(item_entity);
//        Part part;
//    }









        foreach(auto item,_entitylist)
        {
            Part part;
            part.part.push_back(item->clone());
            part.partNum=1;
            part.originalEntity.push_back(item);
            foreach(auto item,part.part)
            {
                if(item->getMax().x-item->getMin().x>part.maxW)
                {
                    part.maxW=item->getMax().x-item->getMin().x;
                }

                if(item->getMax().y-item->getMin().y>part.maxH)
                {
                    part.maxH=item->getMax().y-item->getMin().y;
                }

            }
            partList.push_back(part);
        }
    return partList;

}

void ss_PartFrame::addToFactory(QList<Part> _partlist)
{
    //去除重复零件
    //QList<Part*> partlist;
    //获取零件库待排布零件信息

    m_widgetList.clear();

    foreach(auto item_part,_partlist)
    {
        bool eqal=false;
        int row=0;
        while(row<partList->count())
        {
            QListWidgetItem *item=ss_PartFrame::partList->item(row);
            PartPreview* widget=(PartPreview*)ss_PartFrame::partList->itemWidget(item);
            if(widget->m_part == item_part)
            {
                eqal=true;
                //零件数目+1
                //                    int tem= widget->num->text().toInt();
                //                    if(tem<100)
                //                    {
                //                        tem++;
                //                    }

                //                    widget->num->setText(QString::number(tem));
                //                    widget->m_part.partNum=tem;

                //widget->numIncreaseOne();
            }

            row++;
        }
        if(!eqal)
        {
            WidgetItem widgetitem;
            widgetitem.widget=new PartPreview(item_part);
            widgetitem.item=new QListWidgetItem;
            QSize size =  widgetitem.item->sizeHint();
            widgetitem.item->setSizeHint(QSize(size.width(), 100));
            widgetitem.item->setFlags(widgetitem.item->flags() & ~Qt::ItemIsSelectable);
            m_widgetList.push_back(widgetitem);
        }

    }











    //    foreach(auto item, _partlist)
    //    {
    //        WidgetItem widgetitem;
    //        widgetitem.widget=new PartPreview(item);
    //        widgetitem.item=new QListWidgetItem;
    //        QSize size =  widgetitem.item->sizeHint();
    //        widgetitem.item->setSizeHint(QSize(size.width(), 100));
    //        widgetitem.item->setFlags(widgetitem.item->flags() & ~Qt::ItemIsSelectable);
    //        m_widgetList.push_back(widgetitem);
    //    }
    foreach(auto item,m_widgetList)
    {
        partList->addItem(item.item);
        partList->setItemWidget(item.item,item.widget);
    }
}

RS_Entity *ss_PartFrame::getInsider(RS_Entity * _entity,QList<RS_Entity *> _entityList)
{
    QList<RS_Point> keyPoint;
    switch (_entity->rtti()) {
    case RS2::EntityPolyline:
    {
        RS_Polyline* polyline=( RS_Polyline*)_entity;
        polyline->getRefPoints();
    }
    case RS2::EntityCircle:
    {

    }
    default:
    {

    }

    }
    foreach(auto item, _entityList)
    {

    }
}

//bool ss_PartFrame::isInEntity(RS_Point _point, RS_Entity _entity)
//{

//}

void ss_PartFrame::on_btn_selectAll_clicked()
{
    foreach(auto item,m_widgetList)
    {
        item.widget->checkbox->setCheckState(Qt::Checked);
    }
}

void ss_PartFrame::on_btn_selectNon_clicked()
{
    foreach(auto item,m_widgetList)
    {
        item.widget->checkbox->setCheckState(Qt::Unchecked);
    }
}

void ss_PartFrame::on_btn_selectRev_clicked()
{
    foreach(auto item,m_widgetList)
    {
        if(item.widget->checkbox->isChecked())
        {
            item.widget->checkbox->setCheckState(Qt::Unchecked);
        }
        else
        {
            item.widget->checkbox->setCheckState(Qt::Checked);
        }

    }
}

void ss_PartFrame::on_btn_delete_clicked()
{
    foreach(auto item,m_widgetList)
    {

        if(item.widget->checkbox->isChecked())
        {
            ui->partList->removeItemWidget(item.item);
            delete item.item;
            delete item.widget;
            m_widgetList.removeOne(item);

        }
    }

}

Part::Part()
{
    partNum=1;
    partName="Default";


}

Part *Part::clone()
{
    Part* part=new Part;
    foreach(auto item,this->part)
    {
        part->part.push_back(item->clone());
    }
    foreach(auto item,this->originalEntity)
    {
        part->originalEntity.push_back(item);
    }

    foreach(auto item,part->part)
    {
        if(item->getMax().x-item->getMin().x>part->maxW)
        {
            part->maxW=item->getMax().x-item->getMin().x;
        }

        if(item->getMax().y-item->getMin().y>part->maxH)
        {
            part->maxH=item->getMax().y-item->getMin().y;
        }

    }
    return part;

}

RS_Vector Part::getMax()
{
    RS_Vector Max(0,0);
    foreach(auto item,part)
    {
        if(item->getMax().distanceTo(RS_Vector(0,0))>Max.distanceTo(RS_Vector(0,0)))
        {
            Max=item->getMax();
        }
    }
    return Max;

}

RS_Vector Part::getMin()
{
    RS_Vector Min=part[0]->getMin();
    foreach(auto item,part)
    {
        //        if(item->getMin().x<Min.distanceTo(RS_Vector(0,0)))
        //        {
        //            Min=item->getMin();

        //        }
        if(item->getMin().x<Min.x)
        {
            Min.x=item->getMin().x;
        }
        if(item->getMin().y<Min.y)
        {
            Min.y=item->getMin().y;
        }
    }
    return Min;

}

bool Part::operator ==(const Part part) const
{
    int i=0;
    bool arraysEqual = true;
    if(this->originalEntity.count()!=part.originalEntity.count())
    {
        return false;
    }
    while (arraysEqual && i < this->originalEntity.count())
    {
        if (this->originalEntity[i] != part.originalEntity[i])
            arraysEqual = false;
        i++;
    }
    return arraysEqual;
}
