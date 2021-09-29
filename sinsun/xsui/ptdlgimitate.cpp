#include "ptdlgimitate.h"
#include "ui_ptdlgimitate.h"

ptdlgimitate::ptdlgimitate(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ptdlgimitate)
  , m_animation(new ptAnimationThread())
{
    ui->setupUi(this);
    ui->horizontalSliderScheduleContral->setRange(0,1000);
}

ptdlgimitate::~ptdlgimitate()
{
    delete ui;

    delete m_animation;
}

void ptdlgimitate::pt_begin_thread()
{
    m_animation->run();
}

void ptdlgimitate::closeEvent(QCloseEvent *event)
{
    m_animation->stop();
    m_animation->m_cross_line1->setVisible(false);
    m_animation->m_cross_line2->setVisible(false);
}

void ptdlgimitate::on_pushButtonStart_clicked()
{
    m_animation->start();
}

void ptdlgimitate::on_pushButtonTimeOut_clicked()
{
    m_animation->time_out();
}

void ptdlgimitate::on_pushButtonSlowDown_clicked()
{
    --m_speed_contral;
    m_animation->set_speed(m_speed_contral);
}

void ptdlgimitate::on_pushButtonAccelerate_clicked()
{
    ++m_speed_contral;
    m_animation->set_speed(m_speed_contral);
}

void ptdlgimitate::on_pushButtonWalkBorder_clicked()
{

}

void ptdlgimitate::on_pushButtonStop_clicked()
{
    m_animation->stop();
    m_speed_contral=4;
    m_animation->set_speed(m_speed_contral);
}

void ptdlgimitate::on_horizontalSliderScheduleContral_valueChanged(int value)
{
    //m_animation->set_schedule(value/1000.0);
}
