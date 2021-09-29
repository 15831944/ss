#ifndef PTSHOWNCFILE_H
#define PTSHOWNCFILE_H

#include <QThread>
#include <QEventLoop>
#include <QCloseEvent>
#include "qc_applicationwindow.h"
#include "rs_document.h"
#include "ncfile.h"
#include  "QTimer"
#include "pt_algcommon.h"

class QC_ApplicationWindow;

class ptShowNcFile
{
public:
    ptShowNcFile();
    ptShowNcFile(RS_Graphic* _Graphic);

public:
    //准备图层
    void prepare_layers();

    //重新绘制引线
    void redraw_ncfile();

    void set_plank(double,double);

    //绘制所有信息
    //void reshow_all();

    //显示调整,32个比特
     void set_show_or_hide(int controlSingal);

private:
     //生成一个由graphc托管的copy
     RS_Entity* get_entity_copy(std::shared_ptr<RS_Entity> in);


private:
    RS_Graphic* m_graphic;


public:
    std::shared_ptr<ncFileData> m_ncFile;
};


class ptAnimationThread : public QThread
{
public:
     ptAnimationThread()
     {
     };

    ~ptAnimationThread()
     {
//         graphic->removeEntity(m_cross_line1);
//         graphic->removeEntity(m_cross_line2);
     };

public:
    //开始进程
    void run();

public:

    //开始
    void start(){m_time_out=false;m_stopFlag=false;};

    //暂停
    void time_out(){m_time_out=true;};

    //停止
    void stop(){m_stopFlag=true;m_time_out=true;};

    //设置进度,只能设置为0~1
    //TODO
    void set_schedule(double _radio);

    //设置速度0~10档
    void set_speed(int _speed);

public:
    //设置待显示文件
    void set_nc_file(const ncFileData& inPut);

    //清除ncFile删除内存
    void clear_nc_file();

public:
    RS_Line* m_cross_line1=nullptr;
    RS_Line* m_cross_line2=nullptr;

private:
    //多线程操作
    QEventLoop m_event;

    //速度
    int  m_speed=4;

    //停止标识
    bool m_stopFlag=true;

    //暂停标识
    bool m_time_out=true;

    //初始化
    bool isin=false;

    //整体的起点
    RS_Vector m_start_point;

    //待显示的直线段，所有的数据都当做线段来处理
    QList<RS_Line*> m_show_line_pack;

    //绘图指针
    RS_Graphic* graphic;

};

#endif // PTSHOWNCFILE_H
