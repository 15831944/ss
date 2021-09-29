#include "ptdlgcoldpoints.h"
#include "ui_ptdlgcoldpoints.h"
#include "qc_applicationwindow.h"
#include "sinsun/xstest/ncfileprocesscommon.h"
#include "sinsun/xstest/ncsolidprocesscommon.h"

ptdlgcoldpoints::ptdlgcoldpoints(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ptdlgcoldpoints)
{
    ui->setupUi(this);

    ui->doubleSpinBoxMaxAngle->setValue(90);
}

ptdlgcoldpoints::~ptdlgcoldpoints()
{
    delete ui;
}

void ptdlgcoldpoints::prepare_date()
{
    m_processedNcFile.clear();

    for (size_t i=0;i<m_ncFile->m_segment_file.size();++i)
    {
        ncSolidData* tempPtr=&m_ncFile->m_segment_file[i];

        if(tempPtr->m_polyline->isSelected()==true)
        {
            m_processedNcFile.push_back(tempPtr);
        }
    }
}

void ptdlgcoldpoints::on_pushButtonSelectColdPointsManually_clicked()
{
    //TODO
}

void ptdlgcoldpoints::on_pushButtonFinishSelectColdPointsManually_clicked()
{
    //TODO
}

void ptdlgcoldpoints::on_pushButtonSelectColdPointsAutoly_clicked()
{
    prepare_date();

    for (auto it:m_processedNcFile)
    {
        //存储尖锐的角点
        if(ui->checkBoxClodInConers->isChecked()==true&&it->m_polyline->rtti()==RS2::EntityPolyline)
        {
            auto tempRefPoints=it->m_polyline->getRefPoints();
            for (RS_Vector jt:tempRefPoints)
            {
                it->m_cold_points.insert(jt);
            }
        }
        //储存切入点
        if(ui->checkBoxColdInInPoints->isChecked()==true)
        {
            it->m_cold_points.insert(it->m_cutin_point);
        }
    }

    auto app=QC_ApplicationWindow::getAppWindow();
    app->ptrefresh();
}

void ptdlgcoldpoints::on_pushButtonDeleteColdPoints_clicked()
{
    prepare_date();

    //清除所有的冷却点
    for (auto it:m_processedNcFile)
    {
        it->m_cold_points.clear();
    }

    auto app=QC_ApplicationWindow::getAppWindow();
    app->ptrefresh();
}

void ptdlgcoldpoints::on_buttonBox_accepted()
{
    auto app=QC_ApplicationWindow::getAppWindow();
    app->ptrefresh();
}
