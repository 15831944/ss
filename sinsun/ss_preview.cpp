#include "ss_preview.h"
#include "ui_ss_preview.h"


QList<Entity> m_entityList;
QList <QPointF> ss_Preview::keyPoints;
int  ss_Preview::num=0;

ss_Preview* ss_Preview::p;
qreal max_cood_x=0;
qreal max_cood_y=0;
qreal min_cood_x=0;
qreal min_cood_y=0;
int tran_x=0;
int tran_y=0;
QPointF tem_pos=QPointF(0,0);

ss_Preview::ss_Preview(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ss_Preview)
{
    ui->setupUi(this);
    setAutoFillBackground(true);
    QPalette pal = palette();
    pal.setColor(QPalette::Background,Qt::black);
    setPalette(pal);

    //dxfRW dxf("C:/Users/DLUT422/Desktop/Untitled.dxf");
    //ss_Read read(dxf);
    //dxf.read(&read,1);

    p=this;
    tran_x=0;
    tran_y=0;

}

ss_Preview::~ss_Preview()
{
    delete ui;
}

void initiallize()
{
    m_entityList.clear();
    ss_Preview::keyPoints.clear();
    ss_Preview::num=0;
    max_cood_x=0;
    max_cood_y=0;
    min_cood_x=0;
    min_cood_y=0;
    tran_x=0;
    tran_y=0;
    tem_pos=QPointF(0,0);

}

double get_angle(QPointF _start_point, QPointF _end_point, QPointF _o)
{
    double x1=_start_point.rx(),    y1=_start_point.ry(),
            x2=_end_point.rx(),     y2=_end_point.ry(),
            x3=_o.rx(),                    y3=_o.ry();
    double theta = atan2(x1 - x3, y1 - y3) - atan2(x2 - x3, y2 - y3);
    if (theta > M_PI)
        theta -= 2 * M_PI;
    if (theta < -M_PI)
        theta += 2 * M_PI;

    theta = theta * 180.0 / M_PI;

    return theta;
}

void get_center(double x1,double y1,double x2,double y2,double R,double &x,double &y,double angle)
{
    double c1 = (x2*x2 - x1*x1 + y2*y2 - y1*y1) / (2 *(x2 - x1));
    double c2 = (y2 - y1) / (x2 - x1);
    double A = (c2*c2 + 1);
    double B = (2 * x1*c2 - 2 * c1*c2 - 2 * y1);
    double C = x1*x1 - 2 * x1*c1 + c1*c1 + y1*y1 - R*R;
    double y01 = (-B + sqrt(B*B - 4 * A*C)) / (2 * A);
    double x01 = c1 - c2 * y01;

    double y02=(-B - sqrt(B*B - 4 * A*C)) / (2 * A);
    double x02=c1 - c2 * y02;

    if(angle<0)
    {
        qSwap(y01,y02);
        qSwap(x01,x02);
    }
    if(x2>x1)
    {
        y = y01;
        x = x01;
    }
    if(x1>x2)
    {
        y=y02;
        x=x02;
    }

}


void ss_Preview::draw(const QString path)
{
    initiallize();
    QByteArray ba = path.toLatin1();
    char* path_char=ba.data();
    dxfRW dxf(path_char);
    ss_Read read(dxf);
    dxf.read(&read,1);
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

    p->update();



}

void ss_Preview::paintEvent(QPaintEvent *event)
{

    QPainter painter(this);
    QPen pen;
    //pen.setWidthF(3);
    pen.setColor(Qt::red);
    painter.setPen(pen);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setRenderHint(QPainter::TextAntialiasing);
    int W=this->width();
    int H=this->height();
    QRect rect(0,0,W,H);
    painter.setViewport(rect);
    painter.setWindow(0,H,1*W,-1*H);
    painter.translate(tran_x,-tran_y);

    QPointF window_middle=QPointF(W/2,H/2);
    QPointF graph_middle=QPointF(0,0);

    double side_window=qMin(W,H);
    double side_graph=qMax(max_cood_x-min_cood_x,max_cood_y-min_cood_y);
    double scale=side_window/side_graph*0.8;

    graph_middle=QPointF((max_cood_x+min_cood_x)/2*scale,(max_cood_y+min_cood_y)/2*scale);

    pen.setWidthF(qreal(3/scale));
    painter.setPen(pen);
    painter.scale(scale,scale);
    painter.translate((window_middle.x()-graph_middle.x())/scale,(window_middle.y()-graph_middle.y())/scale);



    foreach(auto item,m_entityList)
    {
        if(item.type==LINE)
        {
            painter.drawLine(QLineF(item.startPoint.x(),item.startPoint.y(),item.endPoint.x(),item.endPoint.y()));

        }
        if(item.type==CIRCLE)
        {
            QRectF rect=QRectF(item.o.rx()-item.r,item.o.ry()-item.r,2*item.r,2*item.r);
            //QRectF rect=QRectF(10,10,20,20);
            painter.drawArc(rect,360*16,360*16);
        }
        if(item.type==ARC)
        {
            qDebug()<<item.startAng/M_PI*180<<item.endAng/M_PI*180;
            QRectF rect=QRectF(item.o.rx()-item.r,item.o.ry()-item.r,2*item.r,2*item.r);
            painter.drawArc(rect,-item.startAng*16/M_PI*180,-(item.endAng-item.startAng)*16/M_PI*180);
        }
        if(item.type==POLYLINE)
        {
            QList<Vertex>::iterator it;
            for(it=item.polyLine.begin();it!=item.polyLine.end()-1;it++)
            {
                if(it->bulge==0)//直线段
                {
                    painter.drawLine(QLineF((it+1)->x,(it+1)->y,it->x,it->y));
                }

                else//圆弧
                {
                    double radian = (atan(it->bulge)) *4;
                    //double angle = radian * 180 / M_PI;
                    double angle = radian;
                    QPointF start_point=QPointF(it->x,it->y);
                    QPointF end_point=QPointF((it+1)->x,(it+1)->y);
                    double r= qAbs((sqrt(pow(start_point.x()-end_point.x(),2)+pow(start_point.y()-end_point.y(),2))) /(2*sin(angle/2)));
                    double x,y;
                    get_center(start_point.x(),start_point.y(),end_point.x(),end_point.y(),r,x,y,angle);

                    QRectF rect=QRectF(x-r,y-r,2*r,2*r);
                    double start_theta=get_angle(start_point,QPointF(x+r,y),QPointF(x,y));
                    double end_theta=get_angle(end_point,start_point,QPointF(x,y));
                    painter.drawArc(rect,start_theta*16,end_theta*16);
                }
            }
            if(item.flag)
            {
                painter.drawLine(QLineF(item.polyLine.begin()->x,item.polyLine.begin()->y,(item.polyLine.end()-1)->x,(item.polyLine.end()-1)->y));
            }


        }
    }
}

void ss_Preview::mouseMoveEvent(QMouseEvent *event)
{
    QPointF mouse_current_pos = event->pos();
    if (event->buttons() & Qt::LeftButton)
    {
        tran_x=(mouse_current_pos.x()-mouse_down_pos.x()+tem_pos.x());
        tran_y=(mouse_current_pos.y()-mouse_down_pos.y()+tem_pos.y());
        if (event->buttons() & Qt::LeftButton)
        {
            tran_x=(mouse_current_pos.x()-mouse_down_pos.x()+tem_pos.x());
            tran_y=(mouse_current_pos.y()-mouse_down_pos.y()+tem_pos.y());
        }

    }
    ss_Preview::update();

}

void ss_Preview::mousePressEvent(QMouseEvent *event)
{
    mouse_down_pos = event->pos();
}

void ss_Preview::mouseReleaseEvent(QMouseEvent *event)
{
    mouse_release_pos = event->pos();
    tem_pos=QPointF(tran_x,tran_y);
}

ss_Read::ss_Read(dxfRW &dxfR):dxfR(dxfR)
{

}

void ss_Read::addLine(const DRW_Line &data)
{
    Entity line;
    line.startPoint.setX(data.basePoint.x);
    line.startPoint.setY(data.basePoint.y);
    line.endPoint.setX(data.secPoint.x);
    line.endPoint.setY(data.secPoint.y);
    line.type=LINE;
    m_entityList.push_back(line);

    ss_Preview::keyPoints.push_back(QPointF(data.basePoint.x,data.basePoint.y));
    ss_Preview::keyPoints.push_back(QPointF(data.secPoint.x,data.secPoint.y));
    ss_Preview::num++;


}

void ss_Read::addArc(const DRW_Arc &data)
{
    Entity arc;
    arc.o.setX(data.basePoint.x);
    arc.o.setY(data.basePoint.y);
    arc.r=data.radious;
    arc.startAng=data.staangle;
    arc.endAng=data.endangle;
    arc.type=ARC;
    m_entityList.push_back(arc);
}

void ss_Read::addCircle(const DRW_Circle &data)
{
    Entity circle;
    circle.o.setX(data.basePoint.x);
    circle.o.setY(data.basePoint.y);
    circle.r=data.radious;
    circle.type=CIRCLE;
    m_entityList.push_back(circle);

    ss_Preview::keyPoints.push_back(QPointF(data.basePoint.x-data.radious,data.basePoint.y-data.radious));
    ss_Preview::keyPoints.push_back(QPointF(data.basePoint.x+data.radious,data.basePoint.y-data.radious));
    ss_Preview::keyPoints.push_back(QPointF(data.basePoint.x-data.radious,data.basePoint.y+data.radious));
    ss_Preview::keyPoints.push_back(QPointF(data.basePoint.x+data.radious,data.basePoint.y+data.radious));
    ss_Preview::num++;
}

void ss_Read::addEllipse(const DRW_Ellipse &data)
{
    Entity polyline;
    DRW_Polyline ellipse2line;
    data.toPolyline(&ellipse2line);

    foreach(auto item, ellipse2line.vertlist)
    {
        Vertex vertex;
        vertex.x=item->basePoint.x;
        vertex.y=item->basePoint.y;
        vertex.bulge=item->bulge;
        polyline.polyLine.push_back(vertex);
        ss_Preview::keyPoints.push_back(QPointF(item->basePoint.x,item->basePoint.y));
    }
    polyline.type=POLYLINE;
    polyline.flag=ellipse2line.flags;
    m_entityList.push_back(polyline);
    ss_Preview::num++;

}

void ss_Read::addLWPolyline(const DRW_LWPolyline &data)
{
    Entity polyline;
    foreach(auto item, data.vertlist)
    {
        Vertex vertex;
        vertex.x=item->x;
        vertex.y=item->y;
        vertex.bulge=item->bulge;
        polyline.polyLine.push_back(vertex);
        ss_Preview::keyPoints.push_back(QPointF(item->x,item->y));

    }
    polyline.type=POLYLINE;
    polyline.flag=data.flags;
    m_entityList.push_back(polyline);
    ss_Preview::num++;
}

void ss_Read::addPolyline(const DRW_Polyline &data)
{
    Entity polyline;

    foreach(auto item, data.vertlist)
    {
        Vertex vertex;
        vertex.x=item->basePoint.x;
        vertex.y=item->basePoint.y;
        vertex.bulge=item->bulge;
        polyline.polyLine.push_back(vertex);
        ss_Preview::keyPoints.push_back(QPointF(item->basePoint.x,item->basePoint.y));

    }
    polyline.type=POLYLINE;
    polyline.flag=data.flags;
    m_entityList.push_back(polyline);
    ss_Preview::num++;
}




