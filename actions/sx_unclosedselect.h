#ifndef SX_UNCLOSEDSELECT_H
#define SX_UNCLOSEDSELECT_H

#include "rs_previewactioninterface.h"
#include"rs_entitycontainer.h"
#include <QMouseEvent>
#include"rs_entity.h"
#include"rs_modification.h"
#include "rs_vector.h"
#include"rs_graphicview.h"
#include "rs.h"
#include "rs_actioninterface.h"
#include "rs_selection.h"
#include "rs_dialogfactory.h"

class sx_UnclosedSelect : public RS_PreviewActionInterface
{
public:
    sx_UnclosedSelect(RS_EntityContainer &container, RS_GraphicView &graphicVi
                      ,RS_ActionInterface* actionSelect=nullptr,
                      std::initializer_list<RS2::EntityType> const& entityTypeList=std::initializer_list<RS2::EntityType>{});
    void trigger() override;
    //void init(int status=0) override;
    void keyPressEvent(QKeyEvent* e) override;
    void mouseReleaseEvent(QMouseEvent* e) override;
    void updateMouseCursor() override;

private:
    std::initializer_list<RS2::EntityType> const entityTypeList;
    RS_Entity* en;
    RS_ActionInterface* actionSelect;
};

#endif // SX_UNCLOSEDSELECT_H
