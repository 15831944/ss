#ifndef SX_SMALLSIZESELECT_H
#define SX_SMALLSIZESELECT_H

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
#include "rs_previewactioninterface.h"
#include "QDebug"
#include "rs_commandevent.h"
#include "rs_debug.h"

class sx_SmallSizeSelect : public RS_PreviewActionInterface
{
public:
    sx_SmallSizeSelect(RS_EntityContainer &container, RS_GraphicView &graphicVi
                       ,RS_ActionInterface* actionSelect=nullptr,
                       std::initializer_list<RS2::EntityType> const& entityTypeList=std::initializer_list<RS2::EntityType>{});

    void updateMouseButtonHints() override;
    void commandEvent(RS_CommandEvent* e) override;
    QStringList getAvailableCommands() override;
    void trigger() override;
    void keyPressEvent(QKeyEvent* e) override;
    void mouseReleaseEvent(QMouseEvent* e) override;
    void updateMouseCursor() override;
    void cmdChanged();
//    void init(int status=0) override;

private:
    std::initializer_list<RS2::EntityType> const entityTypeList;
    RS_Entity* en;
    RS_ActionInterface* actionSelect;
    QString cmd;
//    RS_EntityContainer contain;
//    RS_GraphicView *View;
};

#endif // SX_SMALLSIZESELECT_H
