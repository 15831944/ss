#ifndef SS_LAYERCUTITEM_H
#define SS_LAYERCUTITEM_H

#include <QWidget>
#include <QHBoxLayout>
#include <QLabel>

namespace Ui {
class ss_LayerCutItem;
}

/**
 * @brief The ss_LayerCutItem class
 * 每个图层的GUI界面
 */
class ss_LayerCutItem : public QWidget
{
    Q_OBJECT
public:
    explicit ss_LayerCutItem(QWidget *parent = nullptr);
    explicit ss_LayerCutItem(QString name, QColor color, QWidget *parent = nullptr);

    ~ss_LayerCutItem();
    //getter
    void setColor(QColor color);
    void setName(QString name);
    void setEvaporate(bool isEvaporation);
    void setHanding(bool isHanding);
    void setSequence(int sequence);
    void setUsed(bool used);

    //setter
    QColor getColor() const;
    QString getName() const;
    bool getEvaporate() const;
    bool getHanding() const;
    int getSequence() const;
    bool isUsed() const;

private:
    Ui::ss_LayerCutItem *ui;
    QLabel* m_layerColor;
    QLabel* m_layerName;
    QColor m_color;

    bool m_isEvaporation = false;
    bool m_isHanding = true;
    int m_sequence = 1;

    bool m_isUsed = false;
};

#endif // SS_LAYERCUTITEM_H
