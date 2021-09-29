#ifndef SS_PREVIEW_H
#define SS_PREVIEW_H

#include <QWidget>
#include <QTextBrowser>
#include <libdxfrw.h>
#include <fstream>
#include <QDebug>
#include <QString>
#include <QPainter>
#include <QMouseEvent>
#include <qmath.h>



enum ENTITY_TYPE {LINE,CIRCLE,ARC,POLYLINE,ELLIPSE};

struct Vertex{
    double x;
    double y;
    double bulge=0;

};

class Entity
{
public:
    QPointF startPoint=QPointF(0,0);
    QPointF endPoint=QPointF(0,0);
    double startAng=0;
    double endAng=0;
    QPointF o=QPointF(0,0);
    double r=0;
    double subr=0;
    ENTITY_TYPE type;
    QList<Vertex> polyLine;
    bool flag=0;

};



namespace Ui {
class ss_Preview;
}

class ss_Preview : public QWidget
{
    Q_OBJECT

public:
    explicit ss_Preview(QWidget *parent = nullptr);
    ~ss_Preview();
    QTextBrowser * text;
    static void draw(const QString);
    static ss_Preview *p;
    static QList <QPointF> keyPoints;
    static int num;



private:
    Ui::ss_Preview *ui;
    dxfRW *dxf;
    void paintEvent(QPaintEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    QPointF mouse_down_pos;
    QPointF mouse_release_pos;


    QList <QPointF> polylineList;


};


class ss_Read :public DRW_Interface
{
private:
    dxfRW &dxfR;
public:
    ss_Read(dxfRW &dxfW);



    virtual ~ss_Read() override {

    }

    virtual void addHeader(const DRW_Header *data) override {

    }

    virtual void addLType(const DRW_LType &data) override {

    }
    virtual void addPlotSettings(const DRW_PlotSettings *data) override{

    }


    virtual void addLayer(const DRW_Layer &data) override {

    }

    virtual void addDimStyle(const DRW_Dimstyle &data) override {

    }

    virtual void addVport(const DRW_Vport &data) override {

    }

    virtual void addTextStyle(const DRW_Textstyle &data) override {

    }

    virtual void addAppId(const DRW_AppId &data) override {

    }

    virtual void addBlock(const DRW_Block &data) override {

    }

    virtual void setBlock(const int handle) override {

    }

    virtual void endBlock() override {

    }

    virtual void addPoint(const DRW_Point &data) override {

    }

    virtual void addLine(const DRW_Line &data) override;

    virtual void addRay(const DRW_Ray &data) override {

    }

    virtual void addXline(const DRW_Xline &data) override {

    }

    virtual void addArc(const DRW_Arc &data) override;

    virtual void addCircle(const DRW_Circle &data) override;

    virtual void addEllipse(const DRW_Ellipse &data) override;

    virtual void addLWPolyline(const DRW_LWPolyline &data) override;

    virtual void addPolyline(const DRW_Polyline &data) override;

    virtual void addSpline(const DRW_Spline *data) override {

    }

    virtual void addKnot(const DRW_Entity &data) override {

    }

    virtual void addInsert(const DRW_Insert &data) override {

    }

    virtual void addTrace(const DRW_Trace &data) override {

    }

    virtual void add3dFace(const DRW_3Dface &data) override {

    }

    virtual void addSolid(const DRW_Solid &data) override {

    }

    virtual void addMText(const DRW_MText &data) override {

    }

    virtual void addText(const DRW_Text &data) override {

    }

    virtual void addDimAlign(const DRW_DimAligned *data) override {

    }

    virtual void addDimLinear(const DRW_DimLinear *data) override {

    }

    virtual void addDimRadial(const DRW_DimRadial *data) override {

    }

    virtual void addDimDiametric(const DRW_DimDiametric *data) override {

    }

    virtual void addDimAngular(const DRW_DimAngular *data) override {

    }

    virtual void addDimAngular3P(const DRW_DimAngular3p *data) override {

    }

    virtual void addDimOrdinate(const DRW_DimOrdinate *data) override {

    }

    virtual void addLeader(const DRW_Leader *data) override {

    }

    virtual void addHatch(const DRW_Hatch *data) override {

    }

    virtual void addViewport(const DRW_Viewport &data) override {

    }

    virtual void addImage(const DRW_Image *data) override {

    }

    virtual void linkImage(const DRW_ImageDef *data) override {

    }

    virtual void addComment(const char *comment) override {

    }

    virtual void writeHeader(DRW_Header &data) override {

    }

    virtual void writeBlocks() override {

    }

    virtual void writeBlockRecords() override {

    }

    virtual void writeLTypes() override {

    }

    virtual void writeLayers() override {

    }

    virtual void writeTextstyles() override {

    }

    virtual void writeObjects() override
    {

    }

    virtual void writeVports() override {

    }

    virtual void writeDimstyles() override {

    }

    virtual void writeAppId() override {

    }

    virtual void writeEntities() override{

    }
};


#endif // SS_PREVIEW_H
