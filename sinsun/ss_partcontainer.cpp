#include "ss_partcontainer.h"
#include <QVBoxLayout>

ss_PartWidget::ss_PartWidget(const QString& title, QWidget* parent)
    :QDockWidget(title, parent)
    ,mdi_win(nullptr)
    ,part_frame(new ss_PartFrame(this))

{
    auto frame = new QFrame(this);
    auto layout = new QVBoxLayout;
    frame->setLayout(layout);

    layout->setMargin(0);
    button = part_frame->button;
    layout->addWidget(part_frame);
    setWidget(frame);

}
