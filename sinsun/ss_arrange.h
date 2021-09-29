#ifndef SS_ARRANGE_H
#define SS_ARRANGE_H

#include <QDialog>
#include "ss_partframe.h"
#include "partpreview.h"
#include "ss_sortwidget.h"

class PartPreview;

namespace Ui {
class ss_Arrange;
}

class ss_Arrange : public QDialog
{
    Q_OBJECT

public:
    explicit ss_Arrange(QWidget *parent = nullptr);
    ~ss_Arrange();
    QList<RS_Entity*>arrange(QList<Part*>,double,double);
    void draw(QList<RS_Entity*>);
    void drawPlank(double,double);
    RS_Layer* l;
    static QList<RS_Entity *> entitylist;
    static double plankLength;
    static double plankWidth;

private slots:
    void on_pushButton_2_clicked();

    void on_pushButton_clicked();

private:
    Ui::ss_Arrange *ui;
};

#endif // SS_ARRANGE_H
