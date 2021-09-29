#ifndef LICENSE_H
#define LICENSE_H

#include <QDialog>
class QListWidgetItem;
namespace Ui {
class License;
}

class License : public QDialog
{
    Q_OBJECT

public:
    explicit License(QWidget *parent = nullptr);
    ~License();

private:
    Ui::License *ui;
    bool addFile(QString name, QString path);
    void setFile(QString name);

public slots:
    void loadFile(QListWidgetItem* item);
    void buttonClick();
};

#endif // LICENSE_H
