#ifndef SS_ARRANGEWIDGET_H
#define SS_ARRANGEWIDGET_H

#include <QDialog>
#include <QPlainTextEdit>
#include <QDebug>
#include <vector>
#include "rs_graphic.h"
#include <iostream>
#include <cmath>
#include <fstream>
#include <QMenuBar>
#include "lc_simpletests.h"
#include "qc_applicationwindow.h"
#include "rs_graphic.h"
#include "rs_math.h"
#include "rs_arc.h"
#include "rs_block.h"
#include "rs_circle.h"
#include "rs_ellipse.h"
#include "rs_line.h"
#include "rs_dimaligned.h"
#include "rs_dimangular.h"
#include "rs_dimdiametric.h"
#include "rs_dimlinear.h"
#include "rs_dimradial.h"
#include "rs_hatch.h"
#include "rs_image.h"
#include "rs_insert.h"
#include "rs_mtext.h"
#include "rs_point.h"
#include "rs_text.h"
#include "rs_entitycontainer.h"
#include "rs_layer.h"
#include "rs_graphicview.h"
#include "rs_debug.h"
#include "QString"
#include "rs_vector.h"
#include "rs_line.h"
#include "rs_graphicview.h"
#include "qc_mdiwindow.h"
#include "lc_simpletests.h"
#include "rs_block.h"
#include "rs_graphic.h"
#include "rs_arc.h"
#include "rs_circle.h"
#include "rs_insert.h"
#include "rs_math.h"
#include "rs_ellipse.h"
#include "rs_polyline.h"
#include <vector>
#include <rs_painter.h>
#include "ss_partframe.h"

class Part;

namespace Ui {
class ss_ArrangeWidget;
}

class ss_ArrangeWidget : public QDialog
{
    Q_OBJECT

public:
    explicit ss_ArrangeWidget(QWidget *parent = nullptr);
    ~ss_ArrangeWidget();
    static QList<RS_Entity*> m_finalEntityList;//排列结果
    static QList<RS_Entity*> getFinalEntityList();

private slots:
    void on_btn_showInfo_clicked();

    void on_btn_arrange_clicked();


private:
    Ui::ss_ArrangeWidget *ui;
    QList<RS_Entity*> m_EntityList;//所有实体
    QList<RS_Entity*> m_TbaEntityList;//待排列实体
    QList<RS_Entity*> m_PlankEntityList;
    QList<RS_Entity*> m_EntityArrList;//待排列所有实体
    QList<RS_Entity*> ::iterator it;

    double plankLength=100,plankWidth=100;
    RS_Layer* l;
    RS_Layer* p;

    QList<RS_Entity*> arrange(QList<RS_Entity*>,double,double);
    void draw(QList<RS_Entity*>);
    void drawPlank(double, double);

    //QC_ApplicationWindow* appWin;
    //RS_Document* d;
    //RS_Graphic* graphic;

};

#endif // SS_ARRANGEWIDGET_H
