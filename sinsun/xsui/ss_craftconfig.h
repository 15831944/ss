#ifndef SS_CRAFTCONFIG_H
#define SS_CRAFTCONFIG_H

#include <QDialog>
#include "sinsun/ss_layercut.h"

namespace Ui {
class ss_CraftConfig;
}

class ss_CraftConfig : public QDialog
{
    Q_OBJECT

public:
    explicit ss_CraftConfig(ss_LayerCut* layerCut, QWidget *parent = nullptr);
    ~ss_CraftConfig();

    QTabWidget* m_tabWidget;

private:
    Ui::ss_CraftConfig *ui;

    ss_LayerCut* m_layerCut;
};

#endif // SS_CRAFTCONFIG_H
