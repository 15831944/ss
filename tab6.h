#ifndef TAB6_H
#define TAB6_H

#include <QWidget>
#include<QToolButton>
#include<QToolBar>

namespace Ui {
class Tab6;
}

class Tab6 : public QWidget
{
    Q_OBJECT

public:
    explicit Tab6(QWidget *parent = nullptr);
    ~Tab6();
    QToolButton *button1;
//    QToolButton *button2;
    QToolButton *button3;
    QToolButton *button4;
    QToolButton *button5;
    QToolButton *button6;
    QToolButton *button7;
    QToolButton *button8;
    QToolButton *button9;
    QToolButton *button10;
    QToolButton *button11;

private:
    Ui::Tab6 *ui;
};

#endif // TAB6_H
