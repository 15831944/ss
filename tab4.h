#ifndef TAB4_H
#define TAB4_H

#include <QWidget>
#include<QToolButton>
#include<QToolBar>

namespace Ui {
class Tab4;
}

class Tab4 : public QWidget
{
    Q_OBJECT

public:
    explicit Tab4(QWidget *parent = nullptr);
    ~Tab4();
    QToolButton *button1;

private:
    Ui::Tab4 *ui;
};

#endif // TAB4_H
