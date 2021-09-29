#ifndef RS_ACTIONWEILIAN_H
#define RS_ACTIONWEILIAN_H

#include <QDialog>
#include"rs_entitycontainer.h"
#include <QMouseEvent>
#include"rs_entity.h"
#include"rs_modification.h"
#include "rs_vector.h"
#include"rs_graphicview.h"
#include "rs_previewactioninterface.h"
namespace Ui {
class rs_actionweilian;
}

class rs_actionweilian : public QDialog ,public RS_PreviewActionInterface
{
   // Q_OBJECT

public:
    explicit rs_actionweilian(RS_EntityContainer &container, RS_GraphicView &graphicView,QWidget *parent = nullptr);
    ~rs_actionweilian();

private:
    Ui::rs_actionweilian *ui;
};

#endif // RS_ACTIONWEILIAN_H
