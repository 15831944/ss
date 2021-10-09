#include "mdlgexportgcode.h"
#include "ui_mdlgexportgcode.h"

#include "QString"
#include "rs_vector.h"
#include "rs_line.h"
#include "rs_graphicview.h"
#include "qc_mdiwindow.h"
#include "lc_simpletests.h"
#include "rs_block.h"
#include "rs_graphic.h"
#include "rs_arc.h"
#include "rs_circle.h"
#include "rs_insert.h"
#include "rs_math.h"
#include "rs_ellipse.h"
#include "rs_polyline.h"
#include "m_ncfile.h"
#include <QFileDialog>

#include "libdxfrw.h"
#include "libdwgr.h"
#include "rs_filterdxfrw.h"

MDlgExportGCode::MDlgExportGCode(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::MDlgExportGCode)
{
    ui->setupUi(this);
}

MDlgExportGCode::~MDlgExportGCode()
{
    delete ui;
}

void MDlgExportGCode::on_buttonBox_accepted()
{
    auto appWin=QC_ApplicationWindow::getAppWindow();

    MncFileIo conver;
    conver.setData(*appWin->m_result_file_list[appWin->getDocument()]);
    std::string result=conver.generate_gcode();
    QString qstr= QString::fromStdString(result);

    QString fileName = QFileDialog::getSaveFileName(appWin,tr("保存G代码"),"",tr("后处理文件(*.NC)"));
    QFile file(fileName);

    if (!file.open(QIODevice::WriteOnly|QIODevice::Text))
    {
        QMessageBox::critical(this, "critical", tr("文件保存失败！"),QMessageBox::Yes, QMessageBox::Yes);
    }
    else
    {
        QTextStream stream(&file);
        stream << qstr;
        stream.flush();
        file.close();
    }
}
