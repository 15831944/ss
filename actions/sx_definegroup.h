#ifndef SX_DEFINEGROUP_H
#define SX_DEFINEGROUP_H

#include <QAction>
#include <QMouseEvent>
#include "rs_actionblockscreate.h"

#include "rs_creation.h"
#include "rs_dialogfactory.h"
#include "rs_graphicview.h"
#include "rs_graphic.h"
#include "rs_insert.h"
#include "rs_modification.h"
#include "rs_coordinateevent.h"
#include "QDebug"


class sx_DefineGroup: public RS_PreviewActionInterface
{
public:
    sx_DefineGroup();

    enum Status {
        SetReferencePoint,    /**< Setting the reference point. */
                ShowDialog            /**< Showing dialog for name. */
    };

    sx_DefineGroup(RS_EntityContainer& container,
                        RS_GraphicView& graphicView);
    ~sx_DefineGroup() override;

    void init(int status=0) override;

    void trigger() override;

    void mouseMoveEvent(QMouseEvent* e) override;
    void mouseReleaseEvent(QMouseEvent* e) override;

    void coordinateEvent(RS_CoordinateEvent* e) override;


    void updateMouseButtonHints() override;
    void updateMouseCursor() override;

protected:
    std::unique_ptr<RS_Vector> referencePoint;
};

#endif // SX_DEFINEGROUP_H
