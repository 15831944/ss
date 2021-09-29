#include "ss_openfilepre.h"
#include "ui_ss_openfilepre.h"

ss_OpenFilePre::ss_OpenFilePre(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ss_OpenFilePre)
{
    //ss_OpenFilePre::setAttribute(Qt::WA_DeleteOnClose,true);
    ui->setupUi(this);
    main_layout = new QHBoxLayout(this);
    sub_layout = new QVBoxLayout(this);
    file_dialog = new QFileDialog();
    file_dialog->setNameFilter(u8"CAD文件(*.dxf)");
    m_PreWidget = new ss_Preview();
    label= new QLabel(QString("test"));
    sub_layout->addWidget(label,1);
    sub_layout->addStretch();
    sub_layout->addWidget(m_PreWidget,10);
    main_layout->addWidget(file_dialog,1);
    main_layout->addLayout(sub_layout,1);
    this->resize(1300,700);
    connect(file_dialog, SIGNAL(fileSelected(const QString)), this, SLOT(file_selected(const QString)));
    connect(file_dialog, SIGNAL(currentChanged(const QString)), this, SLOT(file_selected(const QString)));
    connect(file_dialog, SIGNAL(accepted()), this, SLOT(onAccepted()));
    connect(file_dialog, SIGNAL(rejected()), this, SLOT(onRejected()));
    
}

ss_OpenFilePre::~ss_OpenFilePre()
{
    delete ui;
    
}

void ss_OpenFilePre::file_selected(QString str)
{
    QFileInfo fileinfo;
    fileinfo = QFileInfo(str);
    QString file_suffix = fileinfo.suffix();
    if(file_suffix == "dxf")
    {
        ss_Preview::draw(str);
        file_name=str;
        if(!ss_Preview::keyPoints.isEmpty());
        {
            double length=0;
            double width=0;
            double max_cood_x,max_cood_y,min_cood_x,min_cood_y;

            max_cood_x=ss_Preview::keyPoints.first().rx();
            max_cood_y=ss_Preview::keyPoints.first().ry();
            min_cood_x=ss_Preview::keyPoints.first().rx();
            min_cood_y=ss_Preview::keyPoints.first().ry();

            foreach(auto item,ss_Preview::keyPoints)
            {
                if(item.rx()<min_cood_x)
                {
                    min_cood_x=item.rx();
                }
                if(item.ry()<min_cood_y)
                {
                    min_cood_y=item.ry();
                }
                if(item.rx()>max_cood_x)
                {
                    max_cood_x=item.rx();
                }
                if(item.ry()>max_cood_y)
                {
                    max_cood_y=item.ry();
                }

            }
            length=max_cood_x-min_cood_x;
            width=max_cood_y-min_cood_y;
            label->setText(QString("information: ").append
                           (QString("    num: ").append
                           (QString::number(ss_Preview::num).append
                           (QString("    size: ").append
                           (QString::number(length)).append(QString("*")).append
                           (QString::number(width))))));
        }

    }



}

void ss_OpenFilePre::onAccepted()
{
    ss_OpenFilePre::close();
    auto appWin= QC_ApplicationWindow::getAppWindow();
    appWin->slotFileOpen(file_name);
}

void ss_OpenFilePre::onRejected()
{
    ss_OpenFilePre::close();
}

