#ifndef SS_SORTWIDGET_H
#define SS_SORTWIDGET_H

#include <QDialog>
#include <rs_entity.h>
#include "ss_arrangewidget.h"
#include <QButtonGroup>
#include <math.h>
#include <sinsun/xstest/ncfile.h>
#include <sinsun/xstest/pt_algsort.h>


namespace Ui {
class ss_SortWidget;
}

class ss_SortWidget : public QDialog
{
    Q_OBJECT

public:
    explicit ss_SortWidget(QWidget *parent = nullptr);
    ~ss_SortWidget();
    static QList<RS_Vector> getVector();
    static void draw(QList<RS_Vector>);
    static QList<RS_Vector> vectorList;
    static RS_Layer *s;
    static void sort(const QList<RS_Entity*>, const int);
    static Pt_AlgSortEntities algsort;

private slots:

    void on_pushButton_clicked();



private:
    Ui::ss_SortWidget *ui;


    //NcFile ncfile;



};

#endif // SS_SORTWIDGET_H
