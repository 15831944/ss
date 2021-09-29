#include "ss_craftconfig.h"
#include "ui_ss_craftconfig.h"


ss_CraftConfig::ss_CraftConfig(ss_LayerCut *layerCut, QWidget *parent):
    QDialog(parent),
    ui(new Ui::ss_CraftConfig),
    m_layerCut(layerCut)
{
    ui->setupUi(this);
    m_tabWidget = ui->tabWidget;
}

ss_CraftConfig::~ss_CraftConfig()
{
    delete ui;
}
