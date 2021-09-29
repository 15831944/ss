#ifndef RS_ACTIONSETWIREDIALOG_H
#define RS_ACTIONSETWIREDIALOG_H

#include <QDialog>
#include<QButtonGroup>
namespace Ui {
class rs_actionsetwiredialog;
}

class rs_actionsetwiredialog : public QDialog
{
    Q_OBJECT

public:
    explicit rs_actionsetwiredialog(QWidget *parent = nullptr);
    ~rs_actionsetwiredialog();

private:
    Ui::rs_actionsetwiredialog *ui;
    QButtonGroup*radiogroup;
private slots:
    void comboboxIntroduceSlot(int index);
    void comboboxExtractionSlot(int index);
    void checkBoxSlot(int state);
    void radioSlot(int id);
    void checkSlot1(int state);
    void checkSlot2(int state);
    void checkSlot3(int state);
    void checkSlot4(int state);

};

#endif // RS_ACTIONSETWIREDIALOG_H
