#ifndef SS_OPENFILEPRE_H
#define SS_OPENFILEPRE_H

#include <QDialog>
#include <QHBoxLayout>
#include <QFileDialog>
#include "ss_preview.h"
#include <ui_ss_preview.h>
#include <QPushButton>
#include <QDebug>
#include <qc_applicationwindow.h>
#include <QDialogButtonBox>
#include <QLabel>

namespace Ui {
class ss_OpenFilePre;
}

class ss_OpenFilePre : public QDialog
{
    Q_OBJECT

public:
    explicit ss_OpenFilePre(QWidget *parent = nullptr);
    ~ss_OpenFilePre();
    QHBoxLayout* main_layout;
    QVBoxLayout* sub_layout;
    ss_Preview* m_PreWidget;
    QFileDialog*  file_dialog ;
    QString file_name;
    QLabel* label;


private:
    Ui::ss_OpenFilePre *ui;



private slots:
    void file_selected(QString);
    void onAccepted();
    void onRejected();




};

#endif // SS_OPENFILEPRE_H
