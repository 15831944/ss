#ifndef SS_PARTFRAME_H
#define SS_PARTFRAME_H

#include <QFrame>
//#include "partpreview.h"
#include "QListWidgetItem"
#include "rs_entity.h"
#include "qc_applicationwindow.h"

class PartPreview;

class Part
{
public:
    Part();
    QList<RS_Entity*> part;
    QList<RS_Entity*> originalEntity;
    int partNum=1;
    QString partName="Default";

    double maxW=-1;
    double maxH=-1;



    Part* clone();
    RS_Vector getMax();
    RS_Vector getMin();

    bool operator == (const Part) const;
};

class WidgetItem
{
public:
    PartPreview *widget;
    QListWidgetItem *item;
    WidgetItem()
    {
        //widget=new PartPreview;
        //item=new QListWidgetItem;
        //QSize size = item->sizeHint();
        //item->setSizeHint(QSize(size.width(), 100));

    }
      bool operator ==(const WidgetItem& p)
      {
          return(this->item==p.item&&this->widget==p.widget);
      }
};


namespace Ui {
class ss_PartFrame;
}

class ss_PartFrame : public QFrame
{
    Q_OBJECT

public:
    explicit ss_PartFrame(QWidget *parent = nullptr);
    ~ss_PartFrame();
    static QList<WidgetItem> m_widgetList;
    static QList<Part> spiltPart(QList<RS_Entity*> entitylist);
    static void addToFactory(QList<Part> partlist);
    static QList<Part> m_partList;
    static QListWidget *partList;
    static RS_Entity * getInsider(RS_Entity * _entity,QList<RS_Entity *> _entityList);
    QToolButton *button;
    QToolButton* returnButton();
    //static bool isInEntity(RS_Point _point, RS_Entity _entity);
private slots:
    void on_btn_selectAll_clicked();

    void on_btn_selectNon_clicked();

    void on_btn_selectRev_clicked();

    void on_btn_delete_clicked();

    void slotClose();

private:
    Ui::ss_PartFrame *ui;
};

#endif // SS_PARTFRAME_H
