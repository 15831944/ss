#include "actionflyingcutnew.h"

actionflyingcutnew::actionflyingcutnew(RS_EntityContainer &container, RS_GraphicView &graphicView,int id) :
    RS_ActionInterface("WeiLian Entities", container, graphicView){
    if(!id){
        fc = new rs_actionflyingcut();
        fc->exec();
    }else{
        fca = new rs_actionflyingcutarc();
        fca->exec();
    }
    finish();

}
