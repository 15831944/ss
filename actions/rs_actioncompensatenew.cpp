#include "rs_actioncompensatenew.h"

rs_actioncompensatenew::rs_actioncompensatenew(RS_EntityContainer &container, RS_GraphicView &graphicView,QWidget *parent) :
    RS_ActionInterface("WeiLian Entities", container, graphicView){
    cps = new rs_actioncompensate(container,graphicView);
    cps->exec();
    finish();
}
