#include "ss_layercut.h"
#include "ui_ss_layercut.h"

#include "qc_applicationwindow.h"
#include "xsui/ss_craftconfig.h"

ss_LayerCut::ss_LayerCut(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ss_LayerCut)
{
    ui->setupUi(this);
    m_layerCutList = ui->listWidget;

    QList<QColor> list_color;
    QList<QString> list_name;
    list_color  <<QColor(77,  255, 77)
                <<QColor(255, 166, 211)
                <<QColor(255, 255, 121)
                <<QColor(255, 166, 166)
                <<QColor(166, 77, 255)
                <<QColor(77, 166, 166)
                <<QColor(255, 166, 121)
                <<QColor(77, 166, 77)
                <<QColor(255, 77, 166);
    list_name   <<"标准切割"
                <<"精细切割"
                <<"打标"
                <<"备用1"
                <<"备用2"
                <<"备用3"
                <<"备用4"
                <<"备用5"
                <<"备用6";

    for(int i=0; i<list_name.size(); i++)
    {
        //初始化图层列表
        ss_LayerCutItem* LayerCutItem = new ss_LayerCutItem(list_name[i], list_color[i]);
        QListWidgetItem* item= new QListWidgetItem();
        QVariant var = QVariant::fromValue(LayerCutItem);
        item->setData(Qt::UserRole,var);
        m_layerCutList->addItem(item);
        m_layerCutList->setItemWidget(item, LayerCutItem);
        item->setSizeHint(QSize(ui->listWidget->sizeHint().width(), 70));
    }

    connect(m_layerCutList, &QListWidget::currentItemChanged, this, &ss_LayerCut::currentLayerChanged);
    connect(ui->pushButton_craft, &QPushButton::clicked, this, &ss_LayerCut::showCraftWindow);
}

ss_LayerCut::~ss_LayerCut()
{
    delete ui;
}

void ss_LayerCut::currentLayerChanged(QListWidgetItem *current, QListWidgetItem *previous)
{
    auto appWin=QC_ApplicationWindow::getAppWindow();
    RS_Document* d = appWin->getDocument();
    RS_Graphic* graphic = (RS_Graphic*)d;
    QList<RS_Entity*> entityList =  graphic->getEntityList();
    QVariant tem_var = current->data(Qt::UserRole);
    ss_LayerCutItem* tem_layerItem = tem_var.value<ss_LayerCutItem*>();
    foreach(RS_Entity* entity,entityList)
    {
        if(entity->isSelected())
        {
            RS_Pen pen;
            pen.setColor(tem_layerItem->getColor());
            entity->setPen(pen);
            entity->m_nc_information.layer_cut = m_layerCutList->currentRow();
        }
    }
    graphic->update();
}

void ss_LayerCut::showCraftWindow()
{
    auto appWin=QC_ApplicationWindow::getAppWindow();

    if(m_craftConfig == NULL)
    {
        m_craftConfig = new ss_CraftConfig(this, appWin);
        m_craftConfig->setFixedSize(800,500);

        connect(m_craftConfig, &QDialog::accepted, this, &ss_LayerCut::accepted);
    }
    m_craftConfig->m_tabWidget->clear();
    QSet<int> used_layer = checkUsedState();
    for(int i=0; i<m_layerCutList->count(); i++)
    {
        if(used_layer.contains(i))
        {
            QVariant tem_var = m_layerCutList->item(i)->data(Qt::UserRole);
            ss_LayerCutItem* tem_layerItem = tem_var.value<ss_LayerCutItem*>();
            QPixmap pix(30,30);
            pix.fill(tem_layerItem->getColor());
            ss_Tab_Widgt* tab_Widget = new ss_Tab_Widgt();
            tab_Widget->m_checkBox_isEvaporation->setChecked(tem_layerItem->getEvaporate());
            tab_Widget->m_checkBox_isHanding->setChecked(tem_layerItem->getHanding());
            tab_Widget->m_comboBox_sequence->setCurrentText(QString::number(tem_layerItem->getSequence()));
            //connect(tab_Widget->m_comboBox_sequence, static_cast<void (QComboBox:: *)(const QString &text)>(&QComboBox::currentIndexChanged), this, &ss_LayerCut::updateSequence);
            m_craftConfig->m_tabWidget->addTab(tab_Widget, QIcon(pix), tem_layerItem->getName());
        }

    }
    m_craftConfig->show();
}

/**
 * @brief ss_LayerCut::updateSequence
 * @param text
 * 处理次序下拉框选择值改变信号，备用
 */
void ss_LayerCut::updateSequence(QString text)
{
//    QComboBox* tem_comboBox_sequence = qobject_cast<QComboBox *>(sender());
//    QWidget* tem_widget = tem_comboBox_sequence->parentWidget();
//    qDebug()<<m_craftConfig->m_tabWidget->indexOf(tem_widget);

//    for(int i=0; i<m_craftConfig->m_tabWidget->count(); i++)
//    {
//        ss_Tab_Widgt* tem_widget = dynamic_cast<ss_Tab_Widgt*>(m_craftConfig->m_tabWidget->widget(i));
//    }
}

void ss_LayerCut::accepted()
{
    //todo 筛选出显示的tab页面
    for(int i=0; i<m_craftConfig->m_tabWidget->count(); i++)
    {
        QString tem_name = m_craftConfig->m_tabWidget->tabText(i);
        ss_Tab_Widgt* tem_widget = dynamic_cast<ss_Tab_Widgt*>(m_craftConfig->m_tabWidget->widget(i));
        bool tem_evaporate = tem_widget->m_checkBox_isEvaporation->checkState();
        bool tem_handing = tem_widget->m_checkBox_isHanding->checkState();
        int tem_sequence = tem_widget->m_comboBox_sequence->currentText().toInt();
        for(int j=0; j<m_layerCutList->count(); j++)
        {

                QVariant tem_var = m_layerCutList->item(j)->data(Qt::UserRole);
                ss_LayerCutItem* tem_layerItem = tem_var.value<ss_LayerCutItem*>();
                if(tem_layerItem->getName() == tem_name)
                {
                    tem_layerItem->setEvaporate(tem_evaporate);
                    tem_layerItem->setHanding(tem_handing);
                    tem_layerItem->setSequence(tem_sequence);
                }

        }
    }

    for(int j=0; j<m_layerCutList->count(); j++)
    {
        QVariant tem_var = m_layerCutList->item(j)->data(Qt::UserRole);
        ss_LayerCutItem* tem_layerItem = tem_var.value<ss_LayerCutItem*>();
        qDebug()<<tem_layerItem->getEvaporate()<<tem_layerItem->getHanding()<<tem_layerItem->getSequence();
    }

    auto appWin=QC_ApplicationWindow::getAppWindow();
    RS_Document* d = appWin->getDocument();
    RS_Graphic* graphic = (RS_Graphic*)d;

    MncFileIo fileIO;
    fileIO.setLayerCut(m_layerCutList);
    fileIO.generate_ncFile(graphic->getEntityList());
}

QSet<int> ss_LayerCut::checkUsedState()
{
    auto appWin=QC_ApplicationWindow::getAppWindow();
    RS_Document* d = appWin->getDocument();
    RS_Graphic* graphic = (RS_Graphic*)d;
    QList<RS_Entity*> entityList =  graphic->getEntityList();
    QSet<int> set_used_index;
    foreach(auto entity, entityList)
    {
        set_used_index<<entity->m_nc_information.layer_cut;
    }
    return set_used_index;
}

ss_Tab_Widgt::ss_Tab_Widgt(QWidget *parent):
        QWidget(parent)
{
    QVBoxLayout* layout_b = new QVBoxLayout();
    QHBoxLayout* layout_s;

    layout_s = new QHBoxLayout();
    QLabel* label_isEvaporation = new QLabel("蒸发去膜");
    m_checkBox_isEvaporation = new QCheckBox();
    layout_s->addWidget(label_isEvaporation);
    layout_s->addWidget(m_checkBox_isEvaporation);
    layout_s->addStretch();
    layout_b->addLayout(layout_s);

    layout_s = new QHBoxLayout();
    QLabel* label_isHanding = new QLabel("加工");
    m_checkBox_isHanding = new QCheckBox();
    m_checkBox_isHanding->setCheckState(Qt::Checked);
    layout_s->addWidget(label_isHanding);
    layout_s->addWidget(m_checkBox_isHanding);
    layout_s->addStretch();
    layout_b->addLayout(layout_s);

    layout_s = new QHBoxLayout();
    QLabel* label_sequence = new QLabel("加工顺序");
    m_comboBox_sequence = new QComboBox(this);
    for(int i = 1; i<=9; i++)
    {
        m_comboBox_sequence->addItem(QString::number(i));
    }
    layout_s->addWidget(label_sequence);
    layout_s->addWidget(m_comboBox_sequence);
    layout_s->addStretch();
    layout_b->addLayout(layout_s);

    this->setLayout(layout_b);
}

ss_Tab_Widgt::~ss_Tab_Widgt()
{

}
