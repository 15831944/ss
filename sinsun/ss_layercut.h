#ifndef SS_LAYERCUT_H
#define SS_LAYERCUT_H

#include <QWidget>
#include "QListWidgetItem"
#include <QDebug>
#include <QCheckBox>
#include <QLineEdit>
#include <QComboBox>
#include <QLabel>

#include "xsui/ss_layercutitem.h"
#include "ncfile/m_ncfile.h"

class ss_CraftConfig;
class ss_Tab_Widgt:public QWidget
{
    Q_OBJECT
public:
    explicit ss_Tab_Widgt(QWidget *parent = nullptr);
    ~ss_Tab_Widgt();

    QCheckBox* m_checkBox_isEvaporation;
    QCheckBox* m_checkBox_isHanding;
    QComboBox* m_comboBox_sequence;


};

namespace Ui {
class ss_LayerCut;
}

class ss_LayerCut : public QWidget
{
    Q_OBJECT

public:
    explicit ss_LayerCut(QWidget *parent = nullptr);
    ~ss_LayerCut();

    QListWidget* m_layerCutList;
    QList<RS_Entity*> get_entity(int index);

private:
    Ui::ss_LayerCut *ui;

    ss_CraftConfig* m_craftConfig = NULL;
    QList<QString> m_sequenceUsed;

    void updateNcFile();

public slots:
    void currentLayerChanged(QListWidgetItem *current);
    void showCraftWindow();
    void updateSequence(QString text);
    void accepted();
    QSet<int> checkUsedState();
};

#endif // SS_LAYERCUT_H
