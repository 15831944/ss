#ifndef RS_ACTIONALIGNLEFT_H
#define RS_ACTIONALIGNLEFT_H
#include "rs_previewactioninterface.h"
#include"rs_entitycontainer.h"
#include <QMouseEvent>
#include"rs_entity.h"
#include"rs_modification.h"
#include "rs_vector.h"
#include"rs_graphicview.h"
#include"lc_undosection.h"
class RS_ActionAlign : public RS_PreviewActionInterface {
    Q_OBJECT

public:
    RS_ActionAlign(RS_EntityContainer& container,
                       RS_GraphicView& graphicView,int id);
    ~RS_ActionAlign() override;
    void mouseReleaseEvent(QMouseEvent* e) override;
    void init(int status=0) override;
    void alignLeft();
    void alignRight();
    void alignHorizontal();
    void alignTop();
    void alignBottom();
    void verticalCenter();
    void centerAlign();

};

#endif // RS_ACTIONALIGNLEFT_H
