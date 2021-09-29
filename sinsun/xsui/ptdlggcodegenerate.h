#ifndef PTDLGGCODEGENERATE_H
#define PTDLGGCODEGENERATE_H

#include <QDialog>
#include <qmessagebox.h>
#include <qdebug.h>
#include "qc_applicationwindow.h"
#include "rs_document.h"

#include "sinsun/xstest/pt_algsort.h"

class QC_ApplicationWindow;

namespace Ui {
class ptdlggcodegenerate;
}

class ptdlggcodegenerate : public QDialog
{
    Q_OBJECT

public:
    explicit ptdlggcodegenerate(QWidget *parent = nullptr);
    ~ptdlggcodegenerate();

    void set_nc_file(const ncFileData& _val)
    {
        m_ncfile=_val;
    }

private slots:
    void on_buttonBox_accepted();

private:
    Ui::ptdlggcodegenerate *ui;

private:

    ncFileData m_ncfile;

    QC_ApplicationWindow* m_AppWin;
};

#endif // PTDLGGCODEGENERATE_H
