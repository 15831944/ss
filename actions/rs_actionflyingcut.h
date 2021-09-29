#ifndef RS_ACTIONFLYINGCUT_H
#define RS_ACTIONFLYINGCUT_H

#include <QDialog>
#include"rs_entitycontainer.h"
#include <QMouseEvent>
#include"rs_entity.h"
#include"rs_modification.h"
#include "rs_vector.h"
#include "QPoint"
#include"rs_graphicview.h"
#include "rs_previewactioninterface.h"
namespace Ui {
class rs_actionflyingcut;
}

class rs_actionflyingcut : public QDialog
{
    Q_OBJECT

public:
    explicit rs_actionflyingcut(QWidget *parent = nullptr);
    ~rs_actionflyingcut();

private:
    Ui::rs_actionflyingcut *ui;
private slots:
    void checkboxSlot(int state);
};

#endif // RS_ACTIONFLYINGCUT_H
