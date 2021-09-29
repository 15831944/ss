#ifndef TEST_DIA_H
#define TEST_DIA_H

#include <QDialog>
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

namespace Ui {
class test_dia;
}

class test_dia : public QDialog
{
    Q_OBJECT

public:
    explicit test_dia(QWidget *parent = nullptr);
    ~test_dia();
    RS_Document* test_getinformation();

    friend class QG_GraphicView;

    void  testfunction();

private slots:
    void on_show_line_clicked();

    void on_reserve_clicked();

    void on_ptn_clean_clicked();

private:
     Ui::test_dia *ui;
     QList<RS_Entity *>::iterator k;

     RS_Vector x;
     RS_Vector y;



     RS_Vector o;
     double r;

     RS_Vector center;
     RS_Vector major;
     RS_Vector ratio;

     std::vector<RS_Vector>ConnectPoint;
     std::vector<RS_Vector>::iterator it;
};

#endif // TEST_DIA_H
