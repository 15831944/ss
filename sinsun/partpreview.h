#ifndef PARTPREVIEW_H
#define PARTPREVIEW_H

#include <QWidget>
#include <QPlainTextEdit>
#include <QLineEdit>
#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QCheckBox>
#include <QPainter>
#include <QDebug>
#include "rs_entity.h"
#include "ss_partframe.h"
#include "rs.h"
#include <time.h>
#include <QLabel>
#include <QtMath>

class Part;

namespace Ui {
class PartPreview;
}

class PartPreview : public QWidget
{
    Q_OBJECT

public:
    explicit PartPreview(Part _part, QWidget *parent = nullptr);

    ~PartPreview();
    QCheckBox* checkbox;
    QLineEdit* num;
    Part m_part;
//    int partNum;
//    QString partName;
    QLineEdit* name;
    QLabel* size;
//    QList<RS_Entity*> partEntity;
    QList <QPointF> keyPoints;
    int max_cood_x=0;
    int max_cood_y=0;
    int min_cood_x=0;
    int min_cood_y=0;




public slots:
    void numIncreaseOne();
    void numDecreaseOne();
    //void editingFinish();
    void numChanged();
    void nameChanged();

private:
    Ui::PartPreview *ui;

};



class PartPic : public QWidget
{
    Q_OBJECT
public:
    explicit PartPic(QWidget *parent = nullptr);
    PartPic(QList<RS_Entity*> _partEntity,PartPreview*);
    ~PartPic(){};
    void paintEvent(QPaintEvent *event);
    QList<RS_Entity*> partEntity;
    QList <QPointF> keyPoints;
    qreal max_cood_x;
    qreal max_cood_y;
    qreal min_cood_x;
    qreal min_cood_y;
    void getEdge(QList<RS_Entity *>);
    double length=0;
    double width=0;
    void draw(QList<RS_Entity*>,QPainter*);


};



#endif // PARTPREVIEW_H
