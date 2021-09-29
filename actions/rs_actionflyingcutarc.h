#ifndef RS_ACTIONFLYINGCUTARC_H
#define RS_ACTIONFLYINGCUTARC_H

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
class rs_actionflyingcutarc;
}

class rs_actionflyingcutarc : public QDialog
{
    Q_OBJECT

public:
    explicit rs_actionflyingcutarc(QWidget *parent = nullptr);
    ~rs_actionflyingcutarc();

private:
    Ui::rs_actionflyingcutarc *ui;
};

#endif // RS_ACTIONFLYINGCUTARC_H
