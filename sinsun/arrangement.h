#ifndef ARRANGEMENT_H
#define ARRANGEMENT_H

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

namespace Ui {
class arrangement;
}

class arrangement : public QDialog
{
    Q_OBJECT

public:
    explicit arrangement(QWidget *parent = nullptr);
    ~arrangement();

private slots:
    void on_arrange_clicked();

    void on_clear_clicked();

private:
    Ui::arrangement *ui;
    double panelLength, panelWidth, object1Length, object1Width, object1Num,
    object2Length, object2Width, object2Num;
    void arrange(RS_Graphic* graphic);
    RS_Vector panelPoint1,panelPoint2,panelPoint3;


    QList<RS_Polyline*>*object;
    QList<RS_Polyline*> ::iterator it_object;

    struct ToBeArranged
    {
        double length;
        double width;
        RS_Polyline *polyline;
    };
    QList<struct ToBeArranged>TBA;
    QList<struct ToBeArranged>::iterator it;

    double maxWidth,maxY;

    QList<RS_Entity*> EntityList;
    QList<RS_Entity*> ::iterator it_entity;
    QList<RS_Polyline> PolylineList;
    QList<RS_Polyline> ::iterator it_polyline;
    int flag = 1;
    RS_Layer* layer2;

};

#endif // ARRANGEMENT_H
