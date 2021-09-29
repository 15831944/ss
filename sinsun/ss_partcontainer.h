#ifndef SS_PARTCONTAINER_H
#define SS_PARTCONTAINER_H

#include <QDockWidget>
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

class ss_PartContainer
{
public:
    ss_PartContainer();
    QList<Part> m_partList;


};

class ss_PartWidget : public QDockWidget
{
    Q_OBJECT

public:
    ss_PartWidget(const QString& title, QWidget* parent = 0);

    QC_MDIWindow* mdi_win;
    QToolButton* button;

protected:
    ss_PartFrame *part_frame;

protected slots:

};

#endif // SS_PARTCONTAINER_H
