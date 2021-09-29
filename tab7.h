#ifndef TAB7_H
#define TAB7_H

#include <QWidget>
#include<QToolButton>
#include<QToolBar>
namespace Ui {
class Tab7;
}

class Tab7 : public QWidget
{
    Q_OBJECT

public:
    explicit Tab7(QWidget *parent = nullptr);
    ~Tab7();
    QToolButton *button1;
    QToolButton *button2;
    QToolButton *button3;
    QToolButton *button4;

private:
    Ui::Tab7 *ui;
};

#endif // TAB7_H
