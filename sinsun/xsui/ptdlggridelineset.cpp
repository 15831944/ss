#include "ptdlggridelineset.h"
#include "ui_ptdlggridelineset.h"
#include "qc_applicationwindow.h"
#include "sinsun/xstest/ncfileprocesscommon.h"
#include "sinsun/xstest/ncsolidprocesscommon.h"
#include<QScrollBar>
#include<QPushButton>
#include<QDialogButtonBox>

ptdlggridelineset::ptdlggridelineset(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ptdlggridelineset)
{
    ui->setupUi(this);
    totalheight = 0;
    ui->groupBoxInLine->hide();
    ui->groupBoxLineSet->hide();
    ui->groupBoxOutLine->hide();
    ui->groupBoxLineLocation->hide();
    //引入部分初始化
    ui->comboBoxInLineType->addItem(QString::fromLocal8Bit("无"));
    ui->comboBoxInLineType->addItem(QString::fromLocal8Bit("直线"));
    ui->comboBoxInLineType->addItem(QString::fromLocal8Bit("圆弧"));
    ui->comboBoxInLineType->addItem(QString::fromLocal8Bit("直线+圆弧"));
    ui->comboBoxInLineType->setCurrentIndex(0);
    ui->doubleSpinBoxInLineLength->setValue(5);

    //引入部分初始化
    ui->comboBoxOutLineType->addItem(QString::fromLocal8Bit("无"));
    ui->comboBoxOutLineType->addItem(QString::fromLocal8Bit("直线"));
    ui->comboBoxOutLineType->addItem(QString::fromLocal8Bit("圆弧"));
    ui->comboBoxOutLineType->addItem(QString::fromLocal8Bit("直线+圆弧"));
    ui->comboBoxOutLineType->setCurrentIndex(0);
    ui->doubleSpinBoxOutLineLength->setValue(5);

    //引线位置初始化
    ui->radioButtonAutoSelect->setChecked(true);
    ui->checkBoxEdgeFirst->setChecked(true);

    //选项初始化
    ui->checkBoxInterfaceCheck->setChecked(true);
    ui->groupBox_9->setStyleSheet("background-color:#393939;border:0;");
    ui->groupBox_10->setStyleSheet("background-color:#393939;border:0;");
    ui->groupBox_11->setStyleSheet("background-color:#393939;border:0;");
    ui->groupBox_12->setStyleSheet("background-color:#393939;border:0;");
    ui->pushButton_9->setStyleSheet("background-color:transparent;color:white;font-size:30px;font-weight:blod");
    ui->pushButton_10->setStyleSheet("background-color:transparent;color:white;font-size:30px;font-weight:blod");
    ui->pushButton_11->setStyleSheet("background-color:transparent;color:white;font-size:30px;font-weight:blod");
    ui->pushButton_12->setStyleSheet("background-color:transparent;color:white;font-size:30px;font-weight:blod");
    ui->label_14->setStyleSheet("background-color:#393939;color:#0ff;font-size:16px;font-weight:blod");
    ui->label_15->setStyleSheet("background-color:#393939;color:#0ff;font-size:16px;font-weight:blod");
    ui->label_12->setStyleSheet("background-color:#393939;color:#0ff;font-size:16px;font-weight:blod");
    ui->label_13->setStyleSheet("background-color:#393939;color:#0ff;font-size:16px;font-weight:blod");
    ui->buttonBox->button(QDialogButtonBox::Ok)->setStyleSheet("background-color:#4E4E4E;color:#0ff;width:80px;height:25px;font-size:18px");
    ui->buttonBox->button(QDialogButtonBox::Cancel)->setStyleSheet("background-color:#4E4E4E;color:#0ff;width:80px;height:25px;font-size:18px");
//    ui->buttonBox->button(QDialogButtonBox::Cancel)->setText("取消");
//    ui->buttonBox->button(QDialogButtonBox::Ok)->setText("确定");
    ui->scrollArea_2->widget()->setStyleSheet("QWidget{background-color:#4E4E4E;border:0}"\
                              "QComboBox{background-color:#393939;border:1px solid black;color:#0ff}");
            ui->scrollArea_2->setStyleSheet("QScrollArea{border:0}");

    ui->scrollArea_2->verticalScrollBar()->setStyleSheet("QScrollBar:vertical{margin:0;width:10px;border-width:0px}\
                                QScrollBar::sub-page:vertical{background-color:#707070;border:0;}\
                                QScrollBar::add-page:vertical{background-color:#707070;border:0;}\
                                 QScrollBar::handle:vertical{margin:0;border:0;background:#292929}\
                                 QScrollBar::sub-line:vertical{image:none;}\
                                 QScrollBar::add-line:vertical{image:none;}");
    connect(ui->pushButton_9,SIGNAL(clicked()),this,SLOT(button1Slot()));
    connect(ui->pushButton_10,SIGNAL(clicked()),this,SLOT(button2Slot()));
    connect(ui->pushButton_11,SIGNAL(clicked()),this,SLOT(button3Slot()));
    connect(ui->pushButton_12,SIGNAL(clicked()),this,SLOT(button4Slot()));
    ui->scrollAreaWidgetContents_2->setMinimumHeight(540);
    ui->scrollAreaWidgetContents_2->setMaximumHeight(540);
    this->setWindowFlag(Qt::FramelessWindowHint);

}

ptdlggridelineset::~ptdlggridelineset()
{
    delete ui;
}

void ptdlggridelineset::on_buttonBox_accepted()
{
    ui->groupBoxInLine->hide();
    ui->groupBoxLineSet->hide();
    ui->groupBoxOutLine->hide();
    ui->groupBoxLineLocation->hide();
    ui->pushButton_9->setText("+");
    ui->pushButton_10->setText("+");
    ui->pushButton_11->setText("+");
    ui->pushButton_12->setText("+");
    totalheight = 0;
    ui->scrollAreaWidgetContents_2->setMaximumHeight(540);
    ui->scrollAreaWidgetContents_2->setMinimumHeight(540);
    ui->scrollArea_2->verticalScrollBar()->update();
    ui->scrollArea_2->verticalScrollBar()->setValue(0);



    /// 准备数据
    prepare_date();


    /// 引线位置修改，修改SolidData切入点的位置！
    /// 0表示未设置，1表示自动设定优先从长边，2表示自动设定优先从顶底，3表示按照图形总长统一设定，4表示只改变类型
    if(ui->radioButtonAutoSelect->isChecked()==true)
    {
        //从长边引入
        if(ui->checkBoxEdgeFirst->isChecked()==true)
        {
            for(auto it:m_processedNcFile)
            {
                ncSolidProcessCommon tempProcesser(*it);
                *it=tempProcesser.caculate_long_middle_refpoints();
            }
        }
        //优先从顶点引入
        if(ui->checkBoxCornerFirst->isChecked()==true)
        {
            for(auto it:m_processedNcFile)
            {
                ncSolidProcessCommon tempProcesser(*it);
                *it=tempProcesser.caculate_coner_refpoints();
            }
        }
    }
    else if(ui->radioButtonAccordLocation->isChecked()==true)
    {
        //按照图形总长的比例引入
        double unifiedSetting=ui->doubleSpinBoxSetLocation->value();
        for(auto it:m_processedNcFile)
        {
            ncSolidProcessCommon tempProcesser(*it);
            *it=tempProcesser.caculate_proportion_refpoints(unifiedSetting);
        }
    }
    else if(ui->radioButtonChangeTypeOnly->isChecked()!=true){}

    /// 设置引入线
    //修改是否存在标识
    if(ui->comboBoxInLineType->currentIndex()==0)
    {
        for(auto it:m_processedNcFile)
        {
            it->m_in_transition_line.m_is_exist=false;
            it->m_g0_line->setEndpoint(it->m_cutin_point);
        }
    }
    else
    {
        guidelineparameter inLineParameter{ui->comboBoxInLineType->currentIndex(),ui->doubleSpinBoxInLineLength->value(),ui->doubleSpinBoxInAngle->value(),ui->doubleSpinBoxInRadius->value()};
        QList<QList<RS_Entity*>> inToInsertEntities=generate_guide_lines(inLineParameter);
        int count=0;
        for(auto it:m_processedNcFile)
        {
            it->m_in_transition_line.m_is_exist=true;
            it->m_in_transition_line.m_guide_entities=inToInsertEntities[count];

            RS_Line* b=dynamic_cast<RS_Line*>(inToInsertEntities[count][0]);//保证第一个是直线
            it->m_g0_line->setEndpoint(b->getStartpoint());//修改G00末端点
            ++count;

            //添加小圆孔
            if(ui->checkBoxAddCircle->isChecked()==true)
            {
                double littleCircleRadius=ui->doubleSpinBoxInCircleRadius->value();

                //G00线末端插入圆孔（圆孔切入位置是否需要调整？）
                RS_Vector tempPosition=it->m_g0_line->getEndpoint();
                auto beginPositon=it->m_in_transition_line.m_guide_entities.begin();

                RS_Circle* tempCircle=new RS_Circle();
                tempCircle->setCenter(tempPosition);
                tempCircle->setRadius(littleCircleRadius);
                RS_Entity* tempEntity=dynamic_cast<RS_Entity*>(tempCircle);
                it->m_in_transition_line.m_guide_entities.insert(beginPositon,tempEntity);

                //TODO修改g00末端点
            }
        }
    }

    /// 设置引出线
    if(ui->comboBoxOutLineType->currentIndex()==0)
    {
        for(int i=0;i<m_processedNcFile.size();++i)
        {
            m_processedNcFile[i]->m_out_transition_line.m_is_exist=false;
            if(m_processedNcFileNext[i])  m_processedNcFileNext[i]->m_g0_line->setStartpoint(m_processedNcFile[i]->m_cutin_point);
        }
    }
    else
    {
        guidelineparameter outLineParameter{ui->comboBoxOutLineType->currentIndex(),ui->doubleSpinBoxOutLineLength->value(),ui->doubleSpinBoxOutAngle->value(),ui->doubleSpinBoxOutRadius->value()};
        QList<QList<RS_Entity*>>  outToInsertEntities=generate_guide_lines(outLineParameter);
        //对于直线加圆弧把圆弧放在直线前面！TODO

        int count=0;
        for(auto it:m_processedNcFile)
        {
            it->m_in_transition_line.m_is_exist=true;
            it->m_in_transition_line.m_guide_entities= outToInsertEntities[count];

            RS_Line* b=dynamic_cast<RS_Line*>(outToInsertEntities[count][0]);//保证最后一个是直线
            it->m_g0_line->setStartpoint(b->getEndpoint());//修改G00起点
            ++count;

            //引出线光开关TODO
            if(ui->checkBoxOffLight->isChecked()==true)
            {

            }
        }

    }

    /// 干涉检查
    if(ui->checkBoxInterfaceCheck->isChecked()==true)
    {
        //TODO干涉检查
    }
    auto app=QC_ApplicationWindow::getAppWindow();
    app->ptrefresh();


}



QList<QList<RS_Entity*>> ptdlggridelineset::generate_guide_lines(const guidelineparameter& val)
{
    QList<QList<RS_Entity*>> result;

    if(val.m_type==1)//直线情况TODO角度
    {
        for(auto it:m_processedNcFile)
        {
            QList<RS_Entity*> tempList;
            RS_Vector offSetVec;
            if(it->m_polyline->rtti()==RS2::EntityCircle)
            {
                RS_Vector tempDir=it->m_cutin_point-it->m_polyline->getCenter();
                tempDir=tempDir/sqrt(tempDir.squared());
                tempDir=tempDir.rotate(-val.m_Angle/180*M_PI);//旋转
                offSetVec=tempDir*val.m_lineLength;
            }
            else if(it->m_polyline->rtti()==RS2::EntityPolyline)
            {
                //RS_Polyline
                auto refVector=RS_Vector(-1,0);
                //refVector=refVector.rotate(RS_Vector(-1,0),offAngle-90);///旋转
                offSetVec=refVector*val.m_lineLength;
            }
            RS_Line* tempLine=new RS_Line(nullptr,it->m_cutin_point-offSetVec,it->m_cutin_point);
            tempList.push_back(tempLine);
            result.push_back(tempList);
        }
    }

    if(val.m_type==2)//圆弧情况
    {
        //TODO
    }

    if(val.m_type==3)//直线+圆弧情况,将之前圆弧和直线的情况合并起来即可
    {
        //TODO
    }

    return result;
}

void ptdlggridelineset::set_yin_cut()
{
    //TODO
}

void ptdlggridelineset::set_yang_cut()
{
    //TODO
}

void ptdlggridelineset::on_comboBoxInLineType_currentIndexChanged(int index)
{
    if(index==2||index==3)
    {
        ui->doubleSpinBoxInRadius->setEnabled(true);
    }
    else
    {
        ui->doubleSpinBoxInRadius->setEnabled(false);
    }
}

void ptdlggridelineset::on_comboBoxOutLineType_currentIndexChanged(int index)
{
    if(index==2||index==3)
    {
        ui->doubleSpinBoxOutRadius->setEnabled(true);
    }
    else
    {
        ui->doubleSpinBoxOutRadius->setEnabled(false);
    }
}

void ptdlggridelineset::button1Slot()
{
    if(ui->groupBoxInLine->isVisible()){
        ui->groupBoxInLine->hide();
        ui->pushButton_9->setText("+");
        totalheight-=ui->groupBoxInLine->height();
    }else{
        ui->groupBoxInLine->show();
        ui->pushButton_9->setText("-");
        totalheight+=ui->groupBoxInLine->height();
    }
    if(200+totalheight>=540){
        ui->scrollAreaWidgetContents_2->setMinimumHeight(200+totalheight);
        ui->scrollAreaWidgetContents_2->setMaximumHeight(200+totalheight);
    }else{
        ui->scrollAreaWidgetContents_2->setMinimumHeight(540);
        ui->scrollAreaWidgetContents_2->setMaximumHeight(540);
    }
}
void ptdlggridelineset::button2Slot()
{
    if(ui->groupBoxOutLine->isVisible()){
        ui->groupBoxOutLine->hide();
        ui->pushButton_10->setText("+");
        totalheight-=ui->groupBoxOutLine->height();
    }else{
        ui->groupBoxOutLine->show();
        ui->pushButton_10->setText("-");
        totalheight+=ui->groupBoxOutLine->height();
    }
    if(200+totalheight>=540){
        ui->scrollAreaWidgetContents_2->setMinimumHeight(200+totalheight);
        ui->scrollAreaWidgetContents_2->setMaximumHeight(200+totalheight);
    }else{
        ui->scrollAreaWidgetContents_2->setMinimumHeight(540);
        ui->scrollAreaWidgetContents_2->setMaximumHeight(540);
    }

}
void ptdlggridelineset::button3Slot()
{
    if(ui->groupBoxLineLocation->isVisible()){
        ui->groupBoxLineLocation->hide();
        ui->pushButton_11->setText("+");
        totalheight-=ui->groupBoxLineLocation->height();
    }else{
        ui->groupBoxLineLocation->show();
        ui->pushButton_11->setText("-");
        totalheight+=ui->groupBoxLineLocation->height();
    }
    if(200+totalheight>=540){
        ui->scrollAreaWidgetContents_2->setMinimumHeight(200+totalheight);
        ui->scrollAreaWidgetContents_2->setMaximumHeight(200+totalheight);
    }else{
        ui->scrollAreaWidgetContents_2->setMinimumHeight(540);
        ui->scrollAreaWidgetContents_2->setMaximumHeight(540);
    }

}
void ptdlggridelineset::button4Slot()
{
    if(ui->groupBoxLineSet->isVisible()){
        ui->groupBoxLineSet->hide();
        ui->pushButton_12->setText("+");
        totalheight-=ui->groupBoxLineSet->height();
    }else{
        ui->groupBoxLineSet->show();
        ui->pushButton_12->setText("-");
        totalheight+=ui->groupBoxLineSet->height();
    }
    if(200+totalheight>=540){
        ui->scrollAreaWidgetContents_2->setMinimumHeight(200+totalheight);
        ui->scrollAreaWidgetContents_2->setMaximumHeight(200+totalheight);
    }else{
        ui->scrollAreaWidgetContents_2->setMinimumHeight(540);
        ui->scrollAreaWidgetContents_2->setMaximumHeight(540);
    }
//    ui->scrollAreaWidgetContents_2->setMinimumHeight(200+totalheight);
//    ui->scrollAreaWidgetContents_2->setMaximumHeight(200+totalheight);
}

void ptdlggridelineset::prepare_date()
{
    m_processedNcFile.clear();
    m_processedNcFileNext.clear();

    for (size_t i=0;i<m_ncFile->m_segment_file.size();++i)
    {
        ncSolidData* tempPtr=&m_ncFile->m_segment_file[i];

        ncSolidData* tempPtrNext=nullptr;
        if(i+1!=m_ncFile->m_segment_file.size())
        {
            tempPtrNext=&m_ncFile->m_segment_file[i+1];
        }

        if(tempPtr->m_polyline->isSelected()==true)
        {
            m_processedNcFile.push_back(tempPtr);
            m_processedNcFileNext.push_back(tempPtrNext);
            //过滤非封闭图形,如果不封闭就去除
            if(ui->checkBoxClosedPicOnly->isChecked()==true&&tempPtr->m_is_closed==false)
            {
                m_processedNcFile.pop_back();
                m_processedNcFileNext.pop_back();
            }
            //过滤内外膜图形
            if(ui->checkBoxOuterMembraneOnly->isChecked()==true&&tempPtr->m_is_inner_membrane==true)
            {
                m_processedNcFile.pop_back();
                 m_processedNcFileNext.pop_back();
            }
            else if(ui->checkBoxInsideMembraneOnly->isChecked()==true&&tempPtr->m_is_inner_membrane==false)
            {
                m_processedNcFile.pop_back();
                m_processedNcFileNext.pop_back();
            }
        }
    }
}
