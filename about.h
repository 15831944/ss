#ifndef ABOUT_H
#define ABOUT_H

#include <QDialog>
#include<QPushButton>
namespace Ui {
class about;
}

class about : public QDialog
{
    Q_OBJECT

public:
    explicit about(QWidget *parent = nullptr);
    ~about();
    void setLabelText(QString text);
    QPushButton *button1;

private:
    Ui::about *ui;
private slots:
    void buttonClick();
    void copySlot();
};

#endif // ABOUT_H
