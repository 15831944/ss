#include "ptdlggcodegenerate.h"
#include "ui_ptdlggcodegenerate.h"
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
#include "sinsun/xstest/ncfileio.h"
#include <QFileDialog>

#include "libdxfrw.h"
#include "libdwgr.h"
#include "rs_filterdxfrw.h"

ptdlggcodegenerate::ptdlggcodegenerate(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ptdlggcodegenerate)
{
    ui->setupUi(this);
}

ptdlggcodegenerate::~ptdlggcodegenerate()
{
    delete ui;
}

void ptdlggcodegenerate::on_buttonBox_accepted()
{
    //获取主窗口
    m_AppWin=QC_ApplicationWindow::getAppWindow();

    ncFileIo conver;
    conver.m_file=m_ncfile;
    std::string result=conver.generate_g_code();
    QString qstr= QString::fromStdString(result);

    QString fileName = QFileDialog::getSaveFileName(m_AppWin,tr("保存G代码"),"",tr("后处理文件(*.NC)"));
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
