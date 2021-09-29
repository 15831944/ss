#ifndef TAB3_H
#define TAB3_H

#include <QWidget>
#include<QToolButton>
#include<QToolBar>

namespace Ui {
class Tab3;
}

class Tab3 : public QWidget
{
    Q_OBJECT

public:
    explicit Tab3(QWidget *parent = nullptr);
    ~Tab3();
    QToolButton *button1;
    QToolButton *button2;
    QToolButton *button3;
    QToolButton *button4;
    QToolButton *button5;
    QToolButton *button6;
    QToolButton *button7;
    QToolButton *button8;
    QToolButton *button9;
    QToolButton *button10;
    QToolButton *button11;
    QToolButton *button12;
    QToolButton *button13;
    QToolButton *button14;
    QToolButton *button15;
    QToolButton *button16;

private:
    Ui::Tab3 *ui;
};

#endif // TAB3_H
