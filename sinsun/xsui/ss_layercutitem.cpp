#include "ss_layercutitem.h"
#include "ui_ss_layercutitem.h"

ss_LayerCutItem::ss_LayerCutItem(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ss_LayerCutItem)
{
    ui->setupUi(this);
    QHBoxLayout* layout= new QHBoxLayout();

    QPixmap pix(30,30);
    pix.fill(QColor(Qt::black));
    m_layerColor = new QLabel();
    m_layerName = new QLabel("标准切割");
    m_layerColor->setPixmap(pix);
    layout->addWidget(m_layerColor);
    layout->addWidget(m_layerName);

    ui->frame->setLayout(layout);
}

ss_LayerCutItem::ss_LayerCutItem(QString name, QColor color, QWidget *parent):
    QWidget(parent),
    ui(new Ui::ss_LayerCutItem),
    m_color(color)
{
    ui->setupUi(this);
    QHBoxLayout* layout= new QHBoxLayout();

    QPixmap pix(30,30);
    pix.fill(color);
    m_layerColor = new QLabel();
    m_layerName = new QLabel(name);
    m_layerColor->setPixmap(pix);
    layout->addWidget(m_layerColor);
    layout->addWidget(m_layerName);

    ui->frame->setLayout(layout);
}

ss_LayerCutItem::~ss_LayerCutItem()
{
    delete ui;
}

void ss_LayerCutItem::setColor(QColor color)
{
    QPixmap pix(30,30);
    pix.fill(color);
    m_color = color;
    m_layerColor->setPixmap(pix);
}

QColor ss_LayerCutItem::getColor() const
{
    return m_color;
}

void ss_LayerCutItem::setName(QString name)
{
    m_layerName->setText(name);
}

void ss_LayerCutItem::setEvaporate(bool isEvaporation)
{
    m_isEvaporation = isEvaporation;
}

void ss_LayerCutItem::setHanding(bool isHanding)
{
    m_isHanding = isHanding;
}

void ss_LayerCutItem::setSequence(int sequence)
{
    m_sequence = sequence;
}

void ss_LayerCutItem::setUsed(bool used)
{
    m_isUsed = used;
}

QString ss_LayerCutItem::getName() const
{
    return m_layerName->text();
}

bool ss_LayerCutItem::getEvaporate() const
{
    return m_isEvaporation;
}

bool ss_LayerCutItem::getHanding() const
{
    return m_isHanding;
}

int ss_LayerCutItem::getSequence() const
{
    return m_sequence;
}

bool ss_LayerCutItem::isUsed() const
{
    return m_isUsed;
}
