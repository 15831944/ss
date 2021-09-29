#include "rs_actionweilian.h"
#include "ui_rs_actionweilian.h"

rs_actionweilian::rs_actionweilian(RS_EntityContainer &container, RS_GraphicView &graphicView,QWidget *parent) :
    QDialog(parent),
    RS_PreviewActionInterface("Align Left Entities", container, graphicView),
    ui(new Ui::rs_actionweilian)
{
    ui->setupUi(this);
    QPalette palette;
    palette.setColor(QPalette::Background, QColor(255, 255, 255));
    ui->label->setAutoFillBackground(true);  //一定要这句，否则不行
    ui->label->setPalette(palette);
    this->exec();
}

rs_actionweilian::~rs_actionweilian()
{
    delete ui;
}
