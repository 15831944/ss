#include "partpreview.h"
#include "ui_partpreview.h"



PartPreview::PartPreview(Part _part, QWidget *parent) :
    QWidget(parent),
    m_part(_part),
    ui(new Ui::PartPreview)
{
    ui->setupUi(this);
    size=new QLabel("test");
    QHBoxLayout *partLayout = new QHBoxLayout(this);
    partLayout->setMargin(0);
    auto check=new QCheckBox();
    auto part=new PartPic(m_part.part,this);
    part->setFixedSize(90,90);
    num = new QLineEdit();
    name= new QLineEdit();
    auto numIncrease = new QPushButton("+");
    auto numDecrease = new QPushButton("-");
    QIntValidator* IntValidator = new QIntValidator;
    IntValidator->setRange(0,100);
    num->setValidator(IntValidator);
    num->setText("1");
    num->setFixedSize(50,25);
    QDateTime current_date_time = QDateTime::currentDateTime();
    name->setText(current_date_time.toString("MMdd-hhmmss"));
    m_part.partName = current_date_time.toString("MMdd-hhmmss");
    //name->setFixedSize(50,25);

    QVBoxLayout *subLayout = new QVBoxLayout();

    subLayout->addStretch();
    subLayout->addWidget(name);
    subLayout->addWidget(size);
    subLayout->addStretch();


    numIncrease->setFixedSize(20,20);
    numDecrease->setFixedSize(20,20);
    partLayout->addWidget(check);
    partLayout->addWidget(part);
    //partLayout->addWidget(name);
    partLayout->addLayout(subLayout);
    partLayout->addStretch();
    partLayout->addWidget(numDecrease);
    partLayout->addWidget(num);
    partLayout->addWidget(numIncrease);
    partLayout->addStretch();
    this->setLayout(partLayout);

    checkbox=check;

    connect(numIncrease,SIGNAL(clicked()),this,SLOT(numIncreaseOne()));
    connect(numDecrease,SIGNAL(clicked()),this,SLOT(numDecreaseOne()));
    connect(num,SIGNAL(editingFinished()),this,SLOT(numChanged()));
    connect(name,SIGNAL(editingFinished()),this,SLOT(nameChanged()));




}

PartPreview::~PartPreview()
{
    delete ui;
}

void PartPreview::numIncreaseOne()
{
    int tem= num->text().toInt();
    if(tem<100)
    {
        tem++;
    }

    num->setText(QString::number(tem));
    m_part.partNum=tem;
}

void PartPreview::numDecreaseOne()
{
    int tem= num->text().toInt();
    if(tem>0)
    {
        tem--;
    }

    num->setText(QString::number(tem));
    m_part.partNum=tem;
}

void PartPreview::numChanged()
{
    int tem=num->text().toInt();
    m_part.partNum=tem;
}

void PartPreview::nameChanged()
{
    QString tem=name->text();
    m_part.partName=tem;
}



//preview window
PartPic::PartPic(QWidget *parent):
    QWidget(parent)
{

}

PartPic::PartPic(QList<RS_Entity *> _partEntity,PartPreview* parent)
{
    partEntity=_partEntity;

    getEdge(partEntity);

    QString str;
    str+=QString::number(length,'f',2);
    str+="×";
    str+=QString::number(width,'f',2);
    parent->size->setText(str);
    if(!keyPoints.isEmpty())
    {
        max_cood_x=keyPoints.first().rx();
        max_cood_y=keyPoints.first().ry();
        min_cood_x=keyPoints.first().rx();
        min_cood_y=keyPoints.first().ry();

        foreach(auto item,keyPoints)
        {
            if(item.rx()<min_cood_x)
            {
                min_cood_x=item.rx();
            }
            if(item.ry()<min_cood_y)
            {
                min_cood_y=item.ry();
            }
            if(item.rx()>max_cood_x)
            {
                max_cood_x=item.rx();
            }
            if(item.ry()>max_cood_y)
            {
                max_cood_y=item.ry();
            }

        }
    }



}

void PartPic::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setRenderHint(QPainter::TextAntialiasing);
    QPen pen;
    //pen.setColor(Qt::red);
    painter.setPen(pen);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setRenderHint(QPainter::TextAntialiasing);
    //int W=this->width();
    //int H=this->height();
    int W=90;
    int H=90;
    QRect rect(0,0,W,H);
    painter.setViewport(rect);
    painter.setWindow(0,H,1*W,-1*H);
    //painter.translate(tran_x,-tran_y);

    QPointF window_middle=QPointF(W/2,H/2);
    QPointF graph_middle=QPointF(0,0);

    // double side_window=qMin(W,H);
    double side_graph=qMax(max_cood_x-min_cood_x,max_cood_y-min_cood_y);
    double scale=90/side_graph*0.9;

    graph_middle=QPointF((max_cood_x+min_cood_x)/2*scale,(max_cood_y+min_cood_y)/2*scale);

    pen.setWidthF(qreal(2/scale));
    painter.setPen(pen);
    painter.scale(scale,scale);
    painter.translate((window_middle.x()-graph_middle.x())/scale,(window_middle.y()-graph_middle.y())/scale);


    //画图
    draw(partEntity,&painter);



}

void PartPic::getEdge(QList<RS_Entity *>)
{

    foreach(auto item, partEntity)
    {
        switch (item->rtti())
        {
        case RS2::EntityLine:
        {
            auto line=(RS_Line*) item;
            keyPoints.push_back(QPointF(line->getStartpoint().x,line->getStartpoint().y));
            keyPoints.push_back(QPointF(line->getEndpoint().x,line->getEndpoint().y));
            length=qAbs(line->getStartpoint().x-line->getEndpoint().x);
            width=qAbs(line->getStartpoint().y-line->getEndpoint().y);

            break;
        }
        case RS2::EntityCircle:
        {
            auto circle=(RS_Circle*) item;
            keyPoints.push_back(QPointF(circle->getCenter().x-circle->getRadius(),circle->getCenter().y-circle->getRadius()));
            keyPoints.push_back(QPointF(circle->getCenter().x+circle->getRadius(),circle->getCenter().y+circle->getRadius()));
            keyPoints.push_back(QPointF(circle->getCenter().x+circle->getRadius(),circle->getCenter().y-circle->getRadius()));
            keyPoints.push_back(QPointF(circle->getCenter().x-circle->getRadius(),circle->getCenter().y+circle->getRadius()));
            length=circle->getRadius();
            width=circle->getRadius();
            break;
        }
        case RS2::EntityEllipse:
        {
            auto ellipse=(RS_Ellipse*)item;
            keyPoints.push_back(QPointF(ellipse->getCenter().x-ellipse->getMajorRadius(),ellipse->getCenter().y-ellipse->getMinorRadius()));
            keyPoints.push_back(QPointF(ellipse->getCenter().x+ellipse->getMajorRadius(),ellipse->getCenter().y+ellipse->getMinorRadius()));
            keyPoints.push_back(QPointF(ellipse->getCenter().x+ellipse->getMajorRadius(),ellipse->getCenter().y-ellipse->getMinorRadius()));
            keyPoints.push_back(QPointF(ellipse->getCenter().x-ellipse->getMajorRadius(),ellipse->getCenter().y+ellipse->getMinorRadius()));
            length= ellipse->getMax().x-ellipse->getMin().x;
            width=ellipse->getMax().y-ellipse->getMin().y;
            break;
        }
        case RS2::EntityPolyline:
        {
            auto polyline=(RS_Polyline*)item;
            foreach(auto item,polyline->getRefPoints().getVector())
            {
               keyPoints.push_back(QPointF(item.x,item.y));
            }
            length= polyline->getMax().x-polyline->getMin().x;
            width=polyline->getMax().y-polyline->getMin().y;
            break;
        }
        default:
        {
            break;
        }

        }

    }
}

void PartPic::draw(QList<RS_Entity *>partEntity, QPainter * painter)
{
    foreach(auto item, partEntity)
    {
        if(item->rtti()==RS2::EntityLine)
        {
            auto line= (RS_Line*)item;
            painter->drawLine(line->getStartpoint().x,line->getStartpoint().y,line->getEndpoint().x,line->getEndpoint().y);
        }

        if(item->rtti()==RS2::EntityCircle)
        {
            auto circle =(RS_Circle*)item;
            QRectF rect=QRectF(circle->getCenter().x-circle->getRadius(),circle->getCenter().y-circle->getRadius(),
                               2*circle->getRadius(),2*circle->getRadius());
            painter->drawArc(rect,360*16,360*16);
        }
        if(item->rtti()==RS2::EntityEllipse)
        {
            auto ellipse =(RS_Ellipse*)item;
            double angle=ellipse->getAngle()*180/M_PI;
            painter->translate(ellipse->getCenter().x,ellipse->getCenter().y);
            painter->rotate(angle);
            //QRectF rect=QRectF(ellipse->getCenter().x-ellipse->getMajorRadius(),ellipse->getCenter().y-ellipse->getMinorRadius(),
            //                  2*ellipse->getMajorRadius(),2*ellipse->getMinorRadius());
            QRectF rect=QRectF(-ellipse->getMajorRadius(),-ellipse->getMinorRadius(),
                               2*ellipse->getMajorRadius(),2*ellipse->getMinorRadius());
            painter->drawEllipse(rect);
            painter->rotate(-angle);
            painter->translate(-ellipse->getCenter().x,-ellipse->getCenter().y);
        }
        if(item->rtti()==RS2::EntityPolyline)
        {
            auto polyline =(RS_Polyline*)item;
            draw(polyline->getEntityList(),painter);

        }

    }
}
