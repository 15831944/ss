#ifndef MDLGEXPORTGCODE_H
#define MDLGEXPORTGCODE_H

#include <QDialog>
#include <qmessagebox.h>
#include <qdebug.h>
#include "qc_applicationwindow.h"
#include "rs_document.h"

namespace Ui {
class MDlgExportGCode;
}

class MDlgExportGCode : public QDialog
{
    Q_OBJECT

public:
    explicit MDlgExportGCode(QWidget *parent = nullptr);
    ~MDlgExportGCode();

private:
    Ui::MDlgExportGCode *ui;

private slots:
    void on_buttonBox_accepted();
};

#endif // MDLGEXPORTGCODE_H
