#include "rs_actionsetwire.h"
rs_actionsetwire::rs_actionsetwire(RS_EntityContainer &container, RS_GraphicView &graphicView) :
    RS_ActionInterface("WeiLian Entities", container, graphicView){
    swd = new rs_actionsetwiredialog();
    swd->exec();

}

