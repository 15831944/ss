/*
**********************************************************************************
**
** This file was created for the LibreCAD project (librecad.org), a 2D CAD program.
**
** Copyright (C) 2015 ravas (github.com/r-a-v-a-s)
**
** This program is free software; you can redistribute it and/or
** modify it under the terms of the GNU General Public License
** as published by the Free Software Foundation; either version 2
** of the License, or (at your option) any later version.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with this program; if not, write to the Free Software
** Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
**
**********************************************************************************
*/

#include "lc_widgetfactory.h"
#include "lc_actionfactory.h"
#include "lc_dockwidget.h"
#include "lc_actiongroupmanager.h"

#include "qg_actionhandler.h"
#include "qg_snaptoolbar.h"
#include "qg_blockwidget.h"
#include "qg_layerwidget.h"
#include "qg_librarywidget.h"
#include "qg_commandwidget.h"
#include "qg_selectionwidget.h"
#include "qg_activelayername.h"
#include "qg_mousewidget.h"
#include "qg_pentoolbar.h"
#include "QSlider"
#include "customiconstyle.h"

#include <QMenu>
#include <QFile>
#include <QMenuBar>
#include <QActionGroup>
#include "QComboBox"
#include <QPushButton>
#include <QToolTip>
#include "sinsun/ss_partcontainer.h"

LC_WidgetFactory::LC_WidgetFactory(QC_ApplicationWindow* main_win,
                                   QMap<QString, QAction*>& action_map,
                                   LC_ActionGroupManager* agm)
    : QObject(nullptr)
    , main_window(main_win)
    , a_map(action_map)
    , ag_manager(agm)
{
    file_actions
        << a_map["FileNew"]
        << a_map["FileNewTemplate"]
        << a_map["FileOpen"]
        << a_map["FileSave"]
        << a_map["FileSaveAs"]
        << a_map["FileSaveAll"];

    line_actions1
            << a_map["DrawLine"]
            << a_map["DrawLineAngle"]
            << a_map["DrawLineHorizontal"]
            << a_map["DrawLineVertical"];
    line_actions2
//            << a_map["DrawLineRectangle"]
            << a_map["DrawLineParallelThrough"]

            << a_map["DrawLineParallel"]
            << a_map["DrawLineBisector"];
    line_actions3
            << a_map["DrawLineTangent1"]
            << a_map["DrawLineTangent2"]
            << a_map["DrawLineOrthTan"]
            << a_map["DrawLineOrthogonal"]
            << a_map["DrawLineRelAngle"];
//            << a_map["DrawLinePolygonCenCor"]
//            << a_map["DrawLinePolygonCenTan"]   //20161226 added by txmy
//            << a_map["DrawLinePolygonCorCor"];

    circle_actions1
            << a_map["DrawCircle"]
            << a_map["DrawCircle2P"]
            << a_map["DrawCircle2PR"]
            << a_map["DrawCircle3P"]
            << a_map["DrawCircleCR"];
    circle_actions2
            << a_map["DrawCircleTan2_1P"]
            << a_map["DrawCircleTan1_2P"]
            << a_map["DrawCircleTan2"]
            << a_map["DrawCircleTan3"];

    curve_actions1
            << a_map["DrawArc"]
            << a_map["DrawArc3P"]
            << a_map["DrawArcTangential"];
    curve_actions2
            << a_map["DrawSpline"]
            << a_map["DrawSplinePoints"];
    curve_actions3
            << a_map["DrawEllipseArcAxis"];
    curve_actions4
            << a_map["DrawLineFree"];

    ellipse_actions
            << a_map["DrawEllipseAxis"]
            << a_map["DrawEllipseFociPoint"]
            << a_map["DrawEllipse4Points"]
            << a_map["DrawEllipseCenter3Points"]
            << a_map["DrawEllipseInscribe"];

    polyline_actions1
            << a_map["DrawPolyline"];
    polyline_actions2
            << a_map["PolylineAdd"]
            << a_map["PolylineAppend"]
            << a_map["PolylineDel"]
            << a_map["PolylineDelBetween"]
            << a_map["PolylineTrim"];
    polyline_actions3
            << a_map["PolylineEquidistant"]
            << a_map["PolylineSegment"];

    spareParts_actions
            << a_map["ImportPart"]   //导入零件
            << a_map["ImportStandardParts"]   //导入标准零件
            << a_map["DeleteAllNestingParts"];  //删除所有排样零件

     board_actions
             << a_map["SetBoard"]   //选中图形设为板材
             << a_map["DeleteLayoutBoard"];  //删除所有排样板材

     layout_actions
             << a_map["AutoLayout"]   //自动排样
             << a_map["ClearLayout"];  //清楚排样结果

    select_actions
            << a_map["SelectAll"]
            << a_map["DeselectAll"]
            << a_map["SelectSingle"]
            << a_map["SelectContour"]
            << a_map["SelectWindow"]
            << a_map["DeselectWindow"]
            << a_map["SelectIntersected"]
            << a_map["DeselectIntersected"]
            << a_map["SelectLayer"]
            << a_map["SelectInvert"]
            << a_map["SelectUnclosedGraphics"]  //选择不封闭图形
            << a_map["SelectAllExternalMolds"]   //选择所有外模
            << a_map["SelectAllInternalMolds"]   //选择所有内模
            << a_map["SelectAllDrawingsSmallerThanTheSpecifiedSize"];    //选择小于指定尺寸的


    select_actions1
            << a_map["SelectAll"]
            << a_map["DeselectAll"];
    select_actions2
            << a_map["SelectSingle"]
            << a_map["SelectContour"]
            << a_map["SelectWindow"]
            << a_map["DeselectWindow"]
            << a_map["SelectIntersected"]
            << a_map["DeselectIntersected"];
    select_actions3
            << a_map["SelectLayer"]
            << a_map["SelectInvert"]
            << a_map["SelectUnclosedGraphics"]  //选择不封闭图形
            << a_map["SelectAllExternalMolds"];   //选择所有外模
    select_actions4
            << a_map["SelectAllInternalMolds"]   //选择所有内模
            << a_map["SelectAllDrawingsSmallerThanTheSpecifiedSize"];    //选择小于指定尺寸的


    group_actions
            << a_map["DefineGroups"]    //定义群组
            << a_map["BreakUpSelectedGroups"]     //打散选中群组
            << a_map["BreakUpAllGroups"]    //打散所有群组
            << a_map["BlocksExplode"];   //炸开图形

    array_actions
            << a_map["MatrixPermutation"]    //矩阵排列
            << a_map["InteractiveArrangement"]     //交互排列
            << a_map["FullOf"];   //布满

    optimization_actions
            << a_map["ModifyCut"]   //曲线分割   原打断
            << a_map["RemoveDuplicateLines"]     //去除重复线
            << a_map["RemoveSmallGraphics"]   //去除小图形
            << a_map["MergeConnectors"];    //合并相连线

    sort_actions
            << a_map["Sort"]   //网格排序
            << a_map["LocalShortestSpaceShift"]     //局部最短空移
            << a_map["ToolDieSequencing"]   //刀模排序
            << a_map["SmallPictureFirst"]   //小图优先
            << a_map["FromTheInsideOut"]   //从内到外
            << a_map["FromLeftToRight"]     //从左到右
            << a_map["FromRightToLeft"]   //从右到左
            << a_map["FromTopToBottom"]    //从上到下
            << a_map["FromBottomToTop"]   //从下到上
            << a_map["Clockwise"]   //顺时针
            << a_map["anti-clockwise"];     //逆时针

    dimension_actions
            << a_map["Aligned"]
            << a_map["DimLinear"]
            << a_map["DimLinearHor"]
            << a_map["DimLinearVer"]
            << a_map["DimRadial"]
            << a_map["DimDiametric"]
            << a_map["DimAngular"]
            << a_map["DimLeader"];

    modify_actions                      //修改
            << a_map["ModifyMove"]
 //           << a_map["ModifyRotate"]
//            << a_map["ModifyScale"]
//            << a_map["ModifyMirror"]
//            << a_map["ModifyMoveRotate"]
//            << a_map["ModifyRotate2"]
//            << a_map["ModifyRevertDirection"]
//            << a_map["ModifyTrim"]
//            << a_map["ModifyTrim2"]
            << a_map["ModifyTrimAmount"]
//            << a_map["ModifyOffset"]
//            << a_map["ModifyBevel"]
//            << a_map["ModifyRound"]
//            << a_map["ModifyStretch"]
            << a_map["ModifyEntity"]
            << a_map["ModifyAttributes"]
            << a_map["ModifyExplodeText"]
            << a_map["ModifyDeleteQuick"];
    align_actions
            << a_map["AlignLeft"]
            << a_map["AlignRight"]
            << a_map["HorizontalAlignment"]
            << a_map["AlignTop"]
            << a_map["AlignBottom"]
            << a_map["Verticalcenter"]
            << a_map["CenterAlignment"];

    info_actions
            << a_map["InfoDist"]   //暂时只保留了点到点的距离
//            << a_map["InfoDist2"]
//            << a_map["InfoAngle"]
            << a_map["InfoTotalLength"];   //选中实体的面积
//            << a_map["InfoArea"];

    layer_actions
            << a_map["LayersDefreezeAll"]
            << a_map["LayersFreezeAll"]
            << a_map["LayersUnlockAll"]
            << a_map["LayersLockAll"]
            << a_map["LayersAdd"]
            << a_map["LayersRemove"]
            << a_map["LayersEdit"]
            << a_map["LayersToggleLock"]
            << a_map["LayersToggleView"]
            << a_map["LayersTogglePrint"]
            << a_map["LayersToggleConstruction"];

    block_actions
            << a_map["BlocksDefreezeAll"]
            << a_map["BlocksFreezeAll"]
            << a_map["BlocksToggleView"]
            << a_map["BlocksAdd"]
            << a_map["BlocksRemove"]
            << a_map["BlocksAttributes"]
            << a_map["BlocksInsert"]
            << a_map["BlocksEdit"]
            << a_map["BlocksSave"]
            << a_map["BlocksCreate"]
            << a_map["BlocksExplode"];

//    pTitleBar = new TitleBar(main_win);
//    qDebug()<<"pTitleBar";
//    qDebug()<<pTitleBar->isVisible();
//    main_win->installEventFilter(pTitleBar);
//    pTitleBar->show();
//    qDebug()<<"pTitleBar";
//    qDebug()<<pTitleBar->isVisible();
//    pTitleBar->setVisible(true);
//    qDebug()<<"pTitleBar";
//    qDebug()<<pTitleBar->isVisible();

//    pTitleBar->raise();
//    qDebug()<<"pTitleBar";
//    qDebug()<<pTitleBar->isVisible();
//    pTitleBar->setStyleSheet("font-weight:bold;font-size:16px;font-family:'YouYuan';");
//    main_win->setWindowTitle("SIASUN SiaCAD 2021");

}


void LC_WidgetFactory::createLeftSidebar(int columns, int icon_size)
{
    dock_part = new LC_DockWidget(main_window);
    dock_part->setObjectName("dock_part");
    dock_part->setWindowTitle(QC_ApplicationWindow::tr("ImportPart library"));
    dock_part->add_actions(part_actions, columns, icon_size);

    QWidget *mywid1 = new QWidget();   //添加一个窗口
    QToolBar* part_toolbar = new QToolBar(QC_ApplicationWindow::tr("Part"), dock_part);
    part_toolbar->setObjectName("part_toolbar");
    QLabel *label = new QLabel(mywid1);
    label->setText("排样:零件");
    QFont font ( "Microsoft YaHei", 10, 75); //第一个属性是字体（微软雅黑），第二个是大小，第三个是加粗（权重是75）
    label->setFont(font);
    label->setStyleSheet("color:#0000ff");
    label->setAlignment(Qt::AlignHCenter);
    QToolButton *tbtn = new QToolButton();  //关闭按钮
    tbtn->setIcon(QIcon(QPixmap(":/extui/close.png")));
    tbtn->setStyleSheet("QToolButton{border-radius:4px; background-color: transparent;}"
                        "QToolButton:hover{border-radius:4px; border: 1px solid QColor(200, 200, 200); background-color: transparent;}");
    connect(tbtn,SIGNAL(clicked()),main_window,SLOT(slotCloseWindows()));  //左侧关闭按钮的槽函数


    QHBoxLayout *HorizontalLayout = new QHBoxLayout(dock_part);
    HorizontalLayout->addWidget(label);
    HorizontalLayout->addWidget(tbtn);
    mywid1->setLayout(HorizontalLayout); //在QWidget上进行布局
    dock_part->setTitleBarWidget(mywid1);  //标题栏上添加控件

    part_toolbar->addAction(a_map["ljSelectAll"]);
    part_toolbar->addAction(a_map["ljSelectAllNo"]);
    part_toolbar->addAction(a_map["Antielection"]);
    part_toolbar->addSeparator();
    part_toolbar->addAction(a_map["ljDelete"]);
    part_toolbar->addSeparator();
    part_toolbar->addAction(a_map["AutoLayout"]);
    part_toolbar->addAction(a_map["JumpFillBottom"]);

    QTabWidget *tabwidget = new QTabWidget();
    tabwidget->setTabPosition(QTabWidget::South);
    QWidget *importpart = new QWidget();   //零件库
    QWidget *board = new QWidget();   //板材库
    QWidget *Scheduledboard = new QWidget();   //已排零件库

    tabwidget->addTab(importpart,tr("零件库"));
    tabwidget->addTab(board,tr("板材库"));
    tabwidget->addTab(Scheduledboard,tr("已排板材库"));

    QVBoxLayout *VerticalLayout = new QVBoxLayout(dock_part);
    VerticalLayout->addWidget(part_toolbar);
    VerticalLayout->addWidget(tabwidget);

    QWidget *mywid2 = new QWidget();
    mywid2->setLayout(VerticalLayout);
    dock_part->setWidget(mywid2);  //窗口内添加控件
    main_window->addDockWidget(Qt::LeftDockWidgetArea, dock_part);
    dock_part->setFeatures(QDockWidget::NoDockWidgetFeatures);

    dock_part->raise();
    dock_part->hide();
}


void LC_WidgetFactory::addSidebar()
{
    dock_part = new LC_DockWidget(main_window);
    dock_part->setObjectName("dock_part");
    dock_part->setWindowTitle(QC_ApplicationWindow::tr("ImportPart library"));
   // dock_part->add_actions(part_actions);

    QWidget *mywid1 = new QWidget();   //添加一个窗口
    QToolBar* part_toolbar = new QToolBar(QC_ApplicationWindow::tr("Part"), dock_part);
    part_toolbar->setObjectName("part_toolbar");
    QLabel *label = new QLabel(mywid1);
    label->setText("排样:零件");
    QFont font ( "Microsoft YaHei", 10, 75); //第一个属性是字体（微软雅黑），第二个是大小，第三个是加粗（权重是75）
    label->setFont(font);
    label->setStyleSheet("color:#0000ff");
    label->setAlignment(Qt::AlignHCenter);
    QToolButton *tbtn = new QToolButton();  //关闭按钮
    tbtn->setIcon(QIcon(QPixmap(":/extui/close.png")));
    tbtn->setStyleSheet("QToolButton{border-radius:4px; background-color: transparent;}"
                        "QToolButton:hover{border-radius:4px; border: 1px solid QColor(200, 200, 200); background-color: transparent;}");
    connect(tbtn,SIGNAL(clicked()),main_window,SLOT(slotCloseWindows()));  //左侧关闭按钮的槽函数


    QHBoxLayout *HorizontalLayout = new QHBoxLayout(dock_part);
    HorizontalLayout->addWidget(label);
    HorizontalLayout->addWidget(tbtn);
    mywid1->setLayout(HorizontalLayout); //在QWidget上进行布局
    dock_part->setTitleBarWidget(mywid1);  //标题栏上添加控件

    part_toolbar->addAction(a_map["ljSelectAll"]);
    part_toolbar->addAction(a_map["ljSelectAllNo"]);
    part_toolbar->addAction(a_map["Antielection"]);
    part_toolbar->addSeparator();
    part_toolbar->addAction(a_map["ljDelete"]);
    part_toolbar->addSeparator();
    part_toolbar->addAction(a_map["AutoLayout"]);
    part_toolbar->addAction(a_map["JumpFillBottom"]);

    QTabWidget *tabwidget = new QTabWidget();
    tabwidget->setTabPosition(QTabWidget::South);
    QWidget *importpart = new QWidget();   //零件库
    QWidget *board = new QWidget();   //板材库
    QWidget *Scheduledboard = new QWidget();   //已排零件库

    tabwidget->addTab(importpart,tr("零件库"));
    tabwidget->addTab(board,tr("板材库"));
    tabwidget->addTab(Scheduledboard,tr("已排板材库"));

    QVBoxLayout *VerticalLayout = new QVBoxLayout(dock_part);
    VerticalLayout->addWidget(part_toolbar);
    VerticalLayout->addWidget(tabwidget);

    QWidget *mywid2 = new QWidget();
    mywid2->setLayout(VerticalLayout);
    dock_part->setWidget(mywid2);  //窗口内添加控件
    main_window->addDockWidget(Qt::LeftDockWidgetArea, dock_part);
    dock_part->setFeatures(QDockWidget::NoDockWidgetFeatures);

    dock_part->raise();
    dock_part->hide();
}





void LC_WidgetFactory::createRightSidebar(QG_ActionHandler* action_handler)
{
    QDockWidget* dock_layer = new QDockWidget(main_window);
    dock_layer->setWindowTitle(QC_ApplicationWindow::tr("图层列表"));

//    dock_layer->setStyleSheet("background-color:red;color:white;");
//    dock_layer->setStyleSheet("QDockWidget::title{background-color:red;color:white;}");
    dock_layer->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    dock_layer->setObjectName("layer_dockwidget");
    layer_widget = new QG_LayerWidget(action_handler, dock_layer, "Layer");
    layer_widget->setFocusPolicy(Qt::NoFocus);
//    layer_widget->setWindowFlags(Qt::FramelessWindowHint);
    connect(layer_widget, SIGNAL(escape()), main_window, SLOT(slotFocus()));
    connect(main_window, SIGNAL(windowsChanged(bool)), layer_widget, SLOT(setEnabled(bool)));
    dock_layer->setWidget(layer_widget);
    dock_layer->setFeatures(QDockWidget::NoDockWidgetFeatures);
 //   dock_layer->setWindowFlags(Qt::FramelessWindowHint);

    QDockWidget* dock_block = new QDockWidget();
  //  dock_block->setWindowFlags(Qt::FramelessWindowHint);
//    dock_block->setWindowTitle(QC_ApplicationWindow::tr("Block List"));
//    dock_block->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
//    dock_block->setObjectName("block_dockwidget");

    block_widget = new QG_BlockWidget(action_handler, dock_block, "Block");

//    block_widget->setFocusPolicy(Qt::NoFocus);
  //  dock_block->setFeatures(QDockWidget::NoDockWidgetFeatures);
//    connect(block_widget, SIGNAL(escape()), main_window, SLOT(slotFocus()));
//    connect(main_window, SIGNAL(windowsChanged(bool)), block_widget, SLOT(setEnabled(bool)));
//    dock_block->setWidget(block_widget);
//    dock_block->setFeatures(QDockWidget::NoDockWidgetFeatures);
 //   block_widget->setWindowFlags(Qt::FramelessWindowHint);
//     block_widget->close();

    QDockWidget* dock_library = new QDockWidget(main_window);
    dock_library->setWindowTitle(QC_ApplicationWindow::tr("Library Browser"));
    dock_library->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    dock_library->setObjectName("library_dockwidget");
//    dock_library->setWindowFlags(Qt::FramelessWindowHint);
    library_widget = new QG_LibraryWidget(dock_library, "Library");
    library_widget->setActionHandler(action_handler);
    library_widget->setFocusPolicy(Qt::NoFocus);
//    library_widget->setWindowFlags(Qt::FramelessWindowHint);
    connect(library_widget, SIGNAL(escape()), main_window, SLOT(slotFocus()));
    connect(main_window, SIGNAL(windowsChanged(bool)),
            (QObject*)library_widget->bInsert, SLOT(setEnabled(bool)));
    dock_library->setWidget(library_widget);
    dock_library->setFeatures(QDockWidget::NoDockWidgetFeatures);


    QDockWidget* dock_command = new QDockWidget(QC_ApplicationWindow::tr("Command line"), main_window);
    dock_command->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    dock_command->setObjectName("command_dockwidget");
    command_widget = new QG_CommandWidget(dock_command, "Command");
    command_widget->setActionHandler(action_handler);
    connect(main_window, SIGNAL(windowsChanged(bool)), command_widget, SLOT(setEnabled(bool)));
    connect(command_widget->leCommand, SIGNAL(escape()), main_window, SLOT(setFocus()));
    dock_command->setWidget(command_widget);
//    connect(dock_command, SIGNAL(dockLocationChanged(Qt::DockWidgetArea)),
//            main_window, SLOT(modifyCommandTitleBar(Qt::DockWidgetArea)));
    dock_command->setFeatures(QDockWidget::NoDockWidgetFeatures);
 //   dock_command->setWindowFlags(Qt::FramelessWindowHint);

    dock_library->setFixedSize(380, 280);
    dock_layer->setFixedSize(380, 260);

    dock_layer->setAutoFillBackground(true);
    dock_layer->setStyleSheet("background-color:#4d4d4d;color:white");
    dock_library->setStyleSheet("background-color:#4d4d4d;color:white");
    dock_command->setStyleSheet("background-color:#4d4d4d;color:white");

    //dock_layer_cut
    QDockWidget* dock_layerCut1 = new QDockWidget(QC_ApplicationWindow::tr("切割图层"), main_window);
    dock_layerCut1->setObjectName("layerCut_dockwidget");
    ss_LayerCut* layerCut = new ss_LayerCut(dock_layerCut1);
    dock_layerCut1->setWidget(layerCut);
    dock_layerCut1->setFeatures(QDockWidget::NoDockWidgetFeatures);


    main_window->addDockWidget(Qt::RightDockWidgetArea, dock_layer);
    main_window->addDockWidget(Qt::RightDockWidgetArea, dock_library);
    main_window->addDockWidget(Qt::RightDockWidgetArea, dock_command);
    main_window->addDockWidget(Qt::LeftDockWidgetArea, dock_layerCut1);


}

void LC_WidgetFactory::createStandardToolbars(QG_ActionHandler* action_handler)
{
    QSizePolicy toolBarPolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);




  /*    QToolBar* sortmanually_toolbar = new QToolBar(QC_ApplicationWindow::tr("SortManually"), main_window);
      sortmanually_toolbar->setObjectName("sortmanually_toolbar");
      sortmanually_toolbar->setSizePolicy(toolBarPolicy);
      sortmanually_toolbar->addAction(a_map["First"]);
      sortmanually_toolbar->addAction(a_map["Forward"]);
      QSlider *slider = new QSlider(Qt::Horizontal);
      slider->setGeometry(0,0,100,100);
      slider->setMinimum(0);
      slider->setMaximum(100);
      slider->setTickPosition(QSlider::TicksBelow);
      slider->setMaximumWidth(100);
      slider->setMinimumWidth(100);
      sortmanually_toolbar->addWidget(slider);
      sortmanually_toolbar->addAction(a_map["Backward"]);
      sortmanually_toolbar->addAction(a_map["Last"]);
      sortmanually_toolbar->setMovable(false);*/





    //文件工具栏，保留了新建，打开，保存
  /*  QToolBar* file_toolbar = new QToolBar(QC_ApplicationWindow::tr("File"), main_window);
    file_toolbar->setSizePolicy(toolBarPolicy);
    file_toolbar->setObjectName("file_toolbar");
    file_toolbar->addActions(file_actions);
//    file_toolbar->addAction(a_map["FilePrint"]);   //打印  打印预览
//    file_toolbar->addAction(a_map["FilePrintPreview"]);
//    file_toolbar->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    file_toolbar->setMovable(false);
*/

    QToolBar* edit_toolbar = new QToolBar(QC_ApplicationWindow::tr("Edit"), main_window);
    edit_toolbar->setSizePolicy(toolBarPolicy);
    edit_toolbar->setObjectName("edit_toolbar");
//    edit_toolbar->addAction(a_map["EditKillAllActions"]);  //选择鼠标
//    edit_toolbar->addSeparator();
    edit_toolbar->addAction(a_map["EditUndo"]);
    edit_toolbar->addAction(a_map["EditRedo"]);
    edit_toolbar->addSeparator();
    edit_toolbar->addAction(a_map["EditCut"]);
    edit_toolbar->addAction(a_map["EditCopy"]);
    edit_toolbar->addAction(a_map["EditPaste"]);
//    edit_toolbar->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    edit_toolbar->setMovable(false);

    QToolBar* order_toolbar = new QToolBar(QC_ApplicationWindow::tr("Order"), main_window);
    order_toolbar->setSizePolicy(toolBarPolicy);
    order_toolbar->setObjectName("order_toolbar");
    order_toolbar->addAction(a_map["OrderTop"]);
    order_toolbar->addAction(a_map["OrderBottom"]);
    order_toolbar->addAction(a_map["OrderRaise"]);
    order_toolbar->addAction(a_map["OrderLower"]);
    order_toolbar->hide();
    order_toolbar->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    order_toolbar->setMovable(false);
order_toolbar->setFixedSize(0,0);
//    QToolBar* view_toolbar = new QToolBar(QC_ApplicationWindow::tr("View"), main_window);
//    view_toolbar->setObjectName("view_toolbar");
//    view_toolbar->setSizePolicy(toolBarPolicy);
////   view_toolbar->addAction(a_map["ViewGrid"]);
////    view_toolbar->addAction(a_map["ViewDraft"]);
////    view_toolbar->addSeparator();
//    view_toolbar->addAction(a_map["ZoomRedraw"]);
//    view_toolbar->addAction(a_map["ZoomIn"]);
//    view_toolbar->addAction(a_map["ZoomOut"]);
//    view_toolbar->addAction(a_map["ZoomAuto"]);
//    view_toolbar->addAction(a_map["ZoomPrevious"]);
//    view_toolbar->addAction(a_map["ZoomWindow"]);
//    view_toolbar->addAction(a_map["ZoomPan"]);
////    view_toolbar->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
//    view_toolbar->setMovable(false);
/*
    QToolBar* technology_toolbar = new QToolBar(QC_ApplicationWindow::tr("Technology"), main_window);
    technology_toolbar->setObjectName("technology_toolbar");
    technology_toolbar->setSizePolicy(toolBarPolicy);
    technology_toolbar->addAction(a_map["SetLeadWire"]);
    technology_toolbar->addAction(a_map["DistinguishBetweenInternalAndExternalModels"]);
    technology_toolbar->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    technology_toolbar->setMovable(false);

    QToolBar* clean_toolbar = new QToolBar(QC_ApplicationWindow::tr("Clean"), main_window);
    clean_toolbar->setObjectName("clean_toolbar");
    clean_toolbar->setSizePolicy(toolBarPolicy);
    clean_toolbar->addAction(a_map["CleanLeadWire"]);
    clean_toolbar->addAction(a_map["CleanWeilian"]);
    clean_toolbar->addAction(a_map["CancellationOfCompensation"]);
    clean_toolbar->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    clean_toolbar->setMovable(false);

    QToolBar* processsetup_toolbar = new QToolBar(QC_ApplicationWindow::tr("Process Setup"), main_window);
    processsetup_toolbar->setObjectName("processsetup_toolbar");
    processsetup_toolbar->setSizePolicy(toolBarPolicy);
    processsetup_toolbar->addAction(a_map["StartingPoint"]);
    processsetup_toolbar->addAction(a_map["Compensate"]);
    processsetup_toolbar->addAction(a_map["VaginalIncision"]);
    processsetup_toolbar->addAction(a_map["YangCut"]);
    processsetup_toolbar->addAction(a_map["Weilian"]);
    processsetup_toolbar->addAction(a_map["AutoWeilian"]);
    processsetup_toolbar->addAction(a_map["ExplodeWeilian"]);
    processsetup_toolbar->addAction(a_map["CoolingPoint"]);
    processsetup_toolbar->addAction(a_map["CleanCoolingPoint"]);
    processsetup_toolbar->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    processsetup_toolbar->setMovable(false);


    QToolBar* layout_toolbar = new QToolBar(QC_ApplicationWindow::tr("Layout"), main_window);
    layout_toolbar->setObjectName("layout_toolbar");
    layout_toolbar->setSizePolicy(toolBarPolicy);
    layout_toolbar->addAction(a_map["AutoLayout"]);
    layout_toolbar->addAction(a_map["ClearLayout"]);
    layout_toolbar->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    layout_toolbar->setMovable(false);

    QToolBar* array_toolbar = new QToolBar(QC_ApplicationWindow::tr("Array"), main_window);
    array_toolbar->setObjectName("array_toolbar");
    array_toolbar->setSizePolicy(toolBarPolicy);
    array_toolbar->addAction(a_map["MatrixPermutation"]);
    array_toolbar->addAction(a_map["InteractiveArrangement"]);
    array_toolbar->addAction(a_map["FullOf"]);
    array_toolbar->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    array_toolbar->setMovable(false);

    QToolBar* group_toolbar = new QToolBar(QC_ApplicationWindow::tr("Group"), main_window);
    group_toolbar->setObjectName("group_toolbar");
    group_toolbar->setSizePolicy(toolBarPolicy);
    group_toolbar->addAction(a_map["DefineGroups"]);
    group_toolbar->addAction(a_map["BreakUpSelectedGroups"]);
    group_toolbar->addAction(a_map["BreakUpAllGroups"]);
    group_toolbar->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    group_toolbar->setMovable(false);

    QToolBar* bridging_toolbar = new QToolBar(QC_ApplicationWindow::tr("Briding"), main_window);
    bridging_toolbar->setObjectName("bridging_toolbar");
    bridging_toolbar->setSizePolicy(toolBarPolicy);
    bridging_toolbar->addAction(a_map["Bridging"]);
    bridging_toolbar->addAction(a_map["CommonEdge"]);
    bridging_toolbar->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    bridging_toolbar->setMovable(false);

    //手动排序
    QToolBar* sortmanually_toolbar = new QToolBar(QC_ApplicationWindow::tr("SortManually"), main_window);
    sortmanually_toolbar->setObjectName("sortmanually_toolbar");
    sortmanually_toolbar->setSizePolicy(toolBarPolicy);
    sortmanually_toolbar->addAction(a_map["First"]);
    sortmanually_toolbar->addAction(a_map["Forward"]);
    QSlider *slider = new QSlider(Qt::Horizontal);
    slider->setGeometry(0,0,100,100);
    slider->setMinimum(0);
    slider->setMaximum(100);
    slider->setTickPosition(QSlider::TicksBelow);
    slider->setMaximumWidth(100);
    slider->setMinimumWidth(100);
    sortmanually_toolbar->addWidget(slider);
    sortmanually_toolbar->addAction(a_map["Backward"]);
    sortmanually_toolbar->addAction(a_map["Last"]);
    sortmanually_toolbar->setMovable(false);
*/


//      QToolBar* file_toolbar = new QToolBar(QC_ApplicationWindow::tr("File"), main_window);
//      file_toolbar->setSizePolicy(toolBarPolicy);
//      file_toolbar->setObjectName("file_toolbar");
//      file_toolbar->addActions(file_actions);
//  //    file_toolbar->addAction(a_map["FilePrint"]);   //打印  打印预览
//  //    file_toolbar->addAction(a_map["FilePrintPreview"]);
//  //    file_toolbar->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
//      file_toolbar->setMovable(false);
//      file_toolbar->resize(20,20);



//    QToolBar* edit_toolbar = new QToolBar(QC_ApplicationWindow::tr("Edit"), main_window);
//    edit_toolbar->setSizePolicy(toolBarPolicy);
//    edit_toolbar->setObjectName("edit_toolbar");
//    edit_toolbar->addAction(a_map["EditKillAllActions"]);  //选择鼠标
//    edit_toolbar->addSeparator();
//    edit_toolbar->addAction(a_map["EditUndo"]);
//    edit_toolbar->addAction(a_map["EditRedo"]);
//    edit_toolbar->addSeparator();
//    edit_toolbar->addAction(a_map["EditCut"]);
//    edit_toolbar->addAction(a_map["EditCopy"]);
//    edit_toolbar->addAction(a_map["EditPaste"]);
//    edit_toolbar->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
//    edit_toolbar->setMovable(false);

   // file_toolbar.addToolBarBreak(Qt::TopToolBarArea);
//    Qt::TopToolBarArea
    snap_toolbar = new QG_SnapToolBar(main_window, action_handler, ag_manager);
    snap_toolbar->setWindowTitle(QC_ApplicationWindow::tr("Snap Selection"));
    snap_toolbar->setSizePolicy(toolBarPolicy);
    snap_toolbar->setObjectName("snap_toolbar" );
    action_handler->set_snap_toolbar(snap_toolbar);
    snap_toolbar->setMovable(false);


    pen_toolbar = new QG_PenToolBar(QC_ApplicationWindow::tr("Pen"), main_window);
    pen_toolbar->setFixedSize(0,0);
    pen_toolbar->setSizePolicy(toolBarPolicy);
    pen_toolbar->setObjectName("pen_toolbar");
    pen_toolbar->setMovable(false);


    options_toolbar = new QToolBar(QC_ApplicationWindow::tr("Tool Options"), main_window);
    options_toolbar->setSizePolicy(toolBarPolicy);
    options_toolbar->setObjectName("options_toolbar");
    options_toolbar->setMovable(false);
//    options_toolbar->setFixedSize(0,0);



    QToolBar* tab_toolbar = new QToolBar(QC_ApplicationWindow::tr("Tab"), main_window);


    fileWindow1 = new QMainWindow;

    fileWindow1->setFixedSize(380,126);//顶部右侧
    fileWindow1->setObjectName("fileWindow1");
    fileWindow1->setStyleSheet("QMainWindow#fileWindow1{color:white;margin-top:26px;top:26px;background-color:#4D4D4D;border-bottom: 6px solid rgb(108,111,114);border-top: 4px solid #666666;}");
//    fileWindow1->setFixedWidth(0);



    fileWindow = new QMainWindow;
    QToolBar *fileBar = new QToolBar;
    QToolButton *fileMenu = new QToolButton;

    fileMenu->setText("文件");
    fileMenu->setToolTip("文件");
    fileMenu->setPopupMode(QToolButton::InstantPopup);
//    fileMenu->addActions(file_actions);
    fileMenu->setFixedSize(84,23);
    fileMenu->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                "QToolButton{color:transparent;border:none;background:url(':/new/prefix1/2030.png')}");
    connect(fileMenu,SIGNAL(clicked()),this,SLOT(slotFile()));

    fileBar->addWidget(fileMenu);

    fileMenu = new QToolButton;
    fileMenu->setFixedSize(85,24);
    fileMenu->setText("新建");
    fileMenu->setDefaultAction(a_map["FileNew"]);
    fileMenu->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                "QToolButton{color:transparent;border:none;background:url(':/new/prefix1/811.png')no-repeat center center}"\
                            "QToolButton::hover{background:url(':/new/prefix1/817.png')no-repeat center center}"\
                            "QToolButton::pressed{background:url(':/new/prefix1/2031.png')no-repeat center center}");
    fileBar->addWidget(fileMenu);

    fileMenu = new QToolButton;
    fileMenu->setFixedSize(85,24);
    fileMenu->setText("打开");
    fileMenu->setDefaultAction(a_map["FileOpen"]);
    fileMenu->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                "QToolButton{color:transparent;border:none;background:url(':/new/prefix1/812.png')no-repeat center center}"\
                            "QToolButton::hover{background:url(':/new/prefix1/818.png')no-repeat center center}"\
                            "QToolButton::pressed{background:url(':/new/prefix1/2032.png')no-repeat center center}");
    fileBar->addWidget(fileMenu);

    fileMenu = new QToolButton;
    fileMenu->setPopupMode(QToolButton::InstantPopup);
//    fileMenu->setIcon(QIcon(":/icons/save.png"));
//    fileMenu->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    fileMenu->setText("保 存");
    fileMenu->setToolTip("保存");
//    tab1_toolbar->addWidget(tool_button);
    fileMenu->addAction(a_map["FileSave"]);
    fileMenu->addAction(a_map["FileSaveAs"]);
    fileMenu->addAction(a_map["FileSaveAll"]);
    fileMenu->setFixedSize(85,24);
    fileMenu->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                "QToolButton{color:transparent;border:none;background:url(':/new/prefix1/813.png')no-repeat center center}"\
                            "QToolButton::hover{background:url(':/new/prefix1/819.png')no-repeat center center}"\
                            "QToolButton::pressed{background:url(':/new/prefix1/2033.png')no-repeat center center}"\
                            "QMenu{background:#292929;color:white}");
    fileBar->addWidget(fileMenu);

    fileBar->setMovable(false);
    fileBar->setObjectName("fileBar");
    fileBar->setFixedSize(85,105);
    fileBar->setStyleSheet("QToolBar#fileBar{color: #fff;background-color: #303030;top:0px;margin-top:0px;}");

      //tool_button->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    fileBar->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    fileWindow->addToolBar(Qt::LeftToolBarArea,fileBar);
    fileWindow->setFixedSize(85,126);//左侧文件界面
    fileWindow->setObjectName("fileWindow");
    fileWindow->setStyleSheet("QMainWindow#fileWindow{color: #fff;background-color:#363636;border-bottom: 6px solid rgb(108,111,114);}");
    tab_toolbar->addWidget(fileWindow);




    QTabWidget *tabToolbar = new QTabWidget;
    QMenu*buttonmenu = new QMenu;
    Tab1 *Tab11 = new Tab1();
    Tab3 *Tab33 = new Tab3();
    Tab4 *Tab44 = new Tab4();
    Tab5 *Tab55 = new Tab5();
    Tab6 *Tab66 = new Tab6();
    Tab7 *Tab77 = new Tab7();

    QToolButton* tool_button = Tab11->button1;
       tool_button->setPopupMode(QToolButton::InstantPopup);
       tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
       tool_button->setToolTip("线段");
       tool_button->setText("");
   //    tool_button->addActions(line_actions);
       buttonmenu = new QMenu;
       buttonmenu->addActions(line_actions1);
       buttonmenu->addSeparator();
       buttonmenu->addActions(line_actions2);
       buttonmenu->addSeparator();
       buttonmenu->addActions(line_actions3);
       tool_button->setMenu(buttonmenu);
       buttonmenu->setStyleSheet("QMenu::separator{background-color:#626262;height:1px}"\
                                 "QMenu{background-color:#292929;color:white;}"\
                                 "QMenu::item:selected{background-color: #484848;}");
       tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                   "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                   "QToolButton{border:none;background:url(':/new/prefix1/1185.png')}"    \
           "QToolButton:hover{background:url(':/new/prefix1/607.png');}"\
           "QToolButton:pressed{background:url(':/new/prefix1/636.png');}"     \
            "QMenu{background:#292929;color:white}" \
           );

       tool_button = Tab11->button2;
       tool_button->setPopupMode(QToolButton::InstantPopup);
       tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
       tool_button->setToolTip("圆");
       tool_button->setText("");
       buttonmenu = new QMenu;
       buttonmenu->addActions(circle_actions1);
       buttonmenu->addSeparator();
       buttonmenu->addActions(circle_actions2);
       tool_button->setMenu(buttonmenu);
       buttonmenu->setStyleSheet("QMenu::separator{background-color:#626262;height:1px}"\
                                 "QMenu{background-color:#292929;color:white;}"\
                                 "QMenu::item:selected{background-color: #484848;}");
   //    tool_button->addActions(circle_actions);
       tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                   "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                   "QToolButton{   \
                       border:none;background:url(':/new/prefix1/1186.png')}"    \
           "QToolButton:hover{background:url(':/new/prefix1/608.png');}"\
           "QToolButton:pressed{background:url(':/new/prefix1/637.png');}" \
           "QMenu{background:#292929;color:white}" );

       tool_button = Tab11->button3;
       tool_button->setPopupMode(QToolButton::InstantPopup);
       tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
       tool_button->setToolTip("曲线");
       tool_button->setText("");
       buttonmenu = new QMenu;
       buttonmenu->addActions(curve_actions1);
       buttonmenu->addSeparator();
       buttonmenu->addActions(curve_actions2);
       buttonmenu->addSeparator();
       buttonmenu->addActions(curve_actions3);
       buttonmenu->addSeparator();
       buttonmenu->addActions(curve_actions4);
       tool_button->setMenu(buttonmenu);
       buttonmenu->setStyleSheet("QMenu::separator{background-color:#626262;height:1px}"\
                                 "QMenu{background-color:#292929;color:white;}"\
                                 "QMenu::item:selected{background-color: #484848;}");
   //    tool_button->addActions(curve_actions);
       tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                   "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                   "QToolButton{   \
                       border:none;background:url(':/new/prefix1/1187.png')}"    \
           "QToolButton:hover{background:url(':/new/prefix1/609.png');}"\
           "QToolButton:pressed{background:url(':/new/prefix1/638.png');}"\
           "QMenu{background:#292929;color:white}");


                   tool_button = Tab11->button4;
                   tool_button->setPopupMode(QToolButton::InstantPopup);
                   tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                   tool_button->setToolTip("多段线");
                   tool_button->setText("");
                   buttonmenu = new QMenu;
                   buttonmenu->addActions(polyline_actions1);
                   buttonmenu->addSeparator();
                   buttonmenu->addActions(polyline_actions2);
                   buttonmenu->addSeparator();
                   buttonmenu->addActions(polyline_actions3);
                   tool_button->setMenu(buttonmenu);
                   buttonmenu->setStyleSheet("QMenu::separator{background-color:#626262;height:1px}"\
                                             "QMenu{background-color:#292929;color:white;}"\
                                             "QMenu::item:selected{background-color: #484848;}");
   //                tool_button->addActions(polyline_actions1);
   //                tool_button->addActions(polyline_actions2);
   //                tool_button->addActions(polyline_actions3);
                   tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                               "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                               "QToolButton{   \
                                   border:none;background:url(':/new/prefix1/1188.png')}"    \
                       "QToolButton:hover{background:url(':/new/prefix1/610.png');}"\
                       "QToolButton:pressed{background:url(':/new/prefix1/639.png');}" \
                     "QMenu{background:#292929;color:white;}"  \
                     "QMenu::item:selected{background-color: #4d4d4d;}");


                   tool_button = Tab11->button5;
                   tool_button->setPopupMode(QToolButton::DelayedPopup);
                   tool_button->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
                   tool_button->setToolTip("点");
                   tool_button->setText("");
                   tool_button->setDefaultAction(a_map["DrawPoint"]);
                   tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                               "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                               "QToolButton{   \
                                   color:transparent;border:none;background:url(':/new/prefix1/1189.png')}"    \
                     "QToolButton:hover{background:url(':/new/prefix1/611.png');}"\
                     "QToolButton:pressed{background:url(':/new/prefix1/640.png');}"\
                     "QMenu{background:#292929;color:white}" );


                   tool_button = Tab11->button6;
                   tool_button->setPopupMode(QToolButton::InstantPopup);
   //                tool_button->setIcon(QIcon(":/icons/ellipse.png"));
                   tool_button->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
                   tool_button->setToolTip("椭圆");
                   tool_button->setText("");
                   tool_button->addActions(ellipse_actions);
                   tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                               "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                               "QToolButton{   \
                                   border:none;background:url(':/new/prefix1/1190.png')}"    \
                     "QToolButton:hover{background:url(':/new/prefix1/612.png');}"\
                     "QToolButton:pressed{background:url(':/new/prefix1/641.png');}"\
                     "QMenu{background:#292929;color:white}" \
                     "QMenu::item:selected{background-color: #484848;}");

                   tool_button = Tab11->button7;
                   tool_button->setPopupMode(QToolButton::InstantPopup);
   //                tool_button->setIcon(QIcon(":/icons/rectangle.png"));
                   tool_button->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
                   tool_button->setToolTip("矩形");
                   tool_button->setText("");
                   buttonmenu = new QMenu;
                   buttonmenu->addAction(a_map["DrawLineRectangle"]);
                   buttonmenu->addSeparator();
                   buttonmenu->addAction(a_map["DrawLinePolygonCenCor"]);
                   buttonmenu->addAction(a_map["DrawLinePolygonCenTan"]);
                   buttonmenu->addAction(a_map["DrawLinePolygonCorCor"]);
                   tool_button->setMenu(buttonmenu);
                   buttonmenu->setStyleSheet("QMenu::separator{background-color:#626262;height:1px}"\
                                             "QMenu{background-color:#292929;color:white;}"\
                                             "QMenu::item:selected{background-color: #484848;}");

   //                tool_button->addAction(a_map["DrawLineRectangle"]);
   //                tool_button->addAction(a_map["DrawLinePolygonCenCor"]);
   //                tool_button->addAction(a_map["DrawLinePolygonCenTan"]);
   //                tool_button->addAction(a_map["DrawLinePolygonCorCor"]);
                   tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                               "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                               "QToolButton{   \
                                   border:none;background:url(':/new/prefix1/1191.png')}"    \
                     "QToolButton:hover{background:url(':/new/prefix1/613.png');}"\
                     "QToolButton:pressed{background:url(':/new/prefix1/642.png');}"\
                     "QMenu{background:#292929;color:white}");

                   tool_button = Tab11->button8;
                   tool_button->setPopupMode(QToolButton::DelayedPopup);
                   tool_button->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
   //                tool_button->setIconSize(QSize(17,11));

                   tool_button->setToolTip("填充");
                   tool_button->setText("");
                   tool_button->setDefaultAction(a_map["DrawHatch"]);
                   tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                               "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                               "QToolButton{   \
                                   color:transparent;border:none;background:url(':/new/prefix1/1192.png')}"    \
                     "QToolButton:hover{background:url(':/new/prefix1/614.png');}"\
                     "QToolButton:pressed{background:url(':/new/prefix1/643.png');}");

                   tool_button = Tab11->button9;
                   tool_button->setPopupMode(QToolButton::InstantPopup);
                   tool_button->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
                   tool_button->setToolTip("移动");
                   tool_button->setText("");
                   tool_button->addAction(a_map["ModifyMoveRotate"]);
                   tool_button->addAction(a_map["ModifyMove"]);
                   tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                               "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                               "QToolButton{   \
                                   border:none;background:url(':/new/prefix1/1193.png')}"    \
                     "QToolButton:hover{background:url(':/new/prefix1/615.png');}"\
                     "QToolButton:pressed{background:url(':/new/prefix1/644.png');}"\
                     "QMenu{background:#292929;color:white}"\
                     "QMenu::item:selected{background-color: #484848;}");

                   tool_button = Tab11->button10;
                   tool_button->setPopupMode(QToolButton::InstantPopup);
                   tool_button->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
                   tool_button->setToolTip("修剪");
                   tool_button->setText("");

                   buttonmenu = new QMenu;
                   buttonmenu->addAction(a_map["ModifyTrim"]);
                   buttonmenu->addAction(a_map["ModifyTrim2"]);
                   buttonmenu->addSeparator();
                   buttonmenu->addAction(a_map["ModifyTrimAmount"]);
                   tool_button->setMenu(buttonmenu);
                   buttonmenu->setStyleSheet("QMenu::separator{background-color:#626262;height:1px}"\
                                             "QMenu{background-color:#292929;color:white;}"\
                                             "QMenu::item:selected{background-color: #484848;}");

   //                tool_button->addAction(a_map["ModifyTrim"]);
   //                tool_button->addAction(a_map["ModifyTrim2"]);
   //                tool_button->addAction(a_map["ModifyTrimAmount"]);
                   tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                               "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                               "QToolButton{   \
                                   border:none;background:url(':/new/prefix1/1194.png')}"    \
                     "QToolButton:hover{background:url(':/new/prefix1/616.png');}"\
                     "QToolButton:pressed{background:url(':/new/prefix1/645.png');}"\
                     "QMenu{background:#292929;color:white}");

                   tool_button = Tab11->button11;
                   tool_button->setPopupMode(QToolButton::InstantPopup);
                   tool_button->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
                   tool_button->setToolTip("圆角");
                   tool_button->setText("");
   //                tool_button->setIcon(QIcon(":/icons/fillet.png"));
                   tool_button->addAction(a_map["ModifyBevel"]);
                   tool_button->addAction(a_map["ModifyRound"]);
                   tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                               "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                               "QToolButton{   \
                                   border:none;background:url(':/new/prefix1/1195.png')}"    \
                     "QToolButton:hover{background:url(':/new/prefix1/617.png');}"\
                     "QToolButton:pressed{background:url(':/new/prefix1/646.png');}"\
                     "QMenu{background:#292929;color:white}"\
                     "QMenu::item:selected{background-color: #484848;}");

                   tool_button = Tab11->button12;
                   tool_button->setPopupMode(QToolButton::DelayedPopup);
                   tool_button->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
                   tool_button->setDefaultAction(a_map["ModifyRotate"]);
                   tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                               "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                               "QToolButton{   \
                                   color:transparent;border:none;background:url(':/new/prefix1/1198.png')}"    \
                     "QToolButton:hover{background:url(':/new/prefix1/618.png');}"\
                     "QToolButton:pressed{background:url(':/new/prefix1/647.png');}");

                   tool_button = Tab11->button13;
                   tool_button->setPopupMode(QToolButton::DelayedPopup);
                   tool_button->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
                   tool_button->setDefaultAction(a_map["ModifyMirror"]);
                   tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                               "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                               "QToolButton{   \
                                   color:transparent;border:none;background:url(':/new/prefix1/1197.png')}"    \
                     "QToolButton:hover{background:url(':/new/prefix1/619.png');}"\
                     "QToolButton:pressed{background:url(':/new/prefix1/648.png');}");

                   tool_button = Tab11->button14;
                   tool_button->setPopupMode(QToolButton::DelayedPopup);
                   tool_button->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
                   tool_button->setDefaultAction(a_map["Size"]);
                   tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                               "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                               "QToolButton{   \
                                   color:transparent;border:none;background:url(':/new/prefix1/1196.png')}"    \
                     "QToolButton:hover{background:url(':/new/prefix1/620.png');}"\
                     "QToolButton:pressed{background:url(':/new/prefix1/649.png');}");

                   tool_button = Tab11->button15;
                   tool_button->setPopupMode(QToolButton::DelayedPopup);
                   tool_button->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
                   tool_button->setDefaultAction(a_map["ModifyStretch"]);
                   tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                               "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                               "QToolButton{   \
                                   color:transparent;border:none;background:url(':/new/prefix1/1199.png')}"    \
                     "QToolButton:hover{background:url(':/new/prefix1/621.png');}"\
                     "QToolButton:pressed{background:url(':/new/prefix1/650.png');}");

                   tool_button = Tab11->button16;
                   tool_button->setPopupMode(QToolButton::DelayedPopup);
                   tool_button->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
                   tool_button->setDefaultAction(a_map["ModifyRevertDirection"]);
                   tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                               "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                               "QToolButton{   \
                                   color:transparent;border:none;background:url(':/new/prefix1/1200.png')}"    \
                     "QToolButton:hover{background:url(':/new/prefix1/622.png');}"\
                     "QToolButton:pressed{background:url(':/new/prefix1/651.png');}");

                   tool_button = Tab11->button17;
                   tool_button->setPopupMode(QToolButton::DelayedPopup);
                   tool_button->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
                   tool_button->setDefaultAction(a_map["ModifyOffset"]);
                   tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                               "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                               "QToolButton{   \
                                   color:transparent;border:none;background:url(':/new/prefix1/1201.png')}"    \
                     "QToolButton:hover{background:url(':/new/prefix1/623.png');}"\
                     "QToolButton:pressed{background:url(':/new/prefix1/652.png');}");

                   tool_button = Tab11->button18;
                   tool_button->setPopupMode(QToolButton::InstantPopup);
                   tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                   tool_button->setDefaultAction(a_map["DrawText"]);
                   tool_button->setIconSize(QSize(30,140));
                   tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                               "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                               "QToolButton{   \
                                   color:transparent;border:none;background:url(':/new/prefix1/1202.png')}"    \
                     "QToolButton:hover{background:url(':/new/prefix1/624.png');}"\
                     "QToolButton:pressed{background:url(':/new/prefix1/653.png');}");

                   tool_button = Tab11->button19;
                   tool_button->setPopupMode(QToolButton::InstantPopup);
                   tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                   tool_button->setDefaultAction(a_map["DimAligned"]);
                   tool_button->setIconSize(QSize(30,140));
                   tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                               "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                               "QToolButton{   \
                                   color:transparent;border:none;background:url(':/new/prefix1/1203.png')}"    \
                     "QToolButton:hover{background:url(':/new/prefix1/625.png');}"\
                     "QToolButton:pressed{background:url(':/new/prefix1/654.png');}");

                       tool_button = Tab11->button20;
                       tool_button->setPopupMode(QToolButton::InstantPopup);
                       tool_button->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
                       tool_button->setToolTip("线性");
                       tool_button->setText("线性");
                       buttonmenu = new QMenu;
                       buttonmenu->addAction(a_map["DimLinear"]);
                       buttonmenu->addAction(a_map["DimLinearHor"]);
                       buttonmenu->addAction(a_map["DimLinearVer"]);
                       buttonmenu->addAction(a_map["DimRadial"]);
                       buttonmenu->addAction(a_map["DimDiametric"]);
                       buttonmenu->addAction(a_map["DimAngular"]);
                       buttonmenu->addSeparator();
                       buttonmenu->addAction(a_map["DimLeader"]);
                       tool_button->setMenu(buttonmenu);
                       buttonmenu->setStyleSheet("QMenu::separator{background-color:#626262;height:1px}"\
                                                 "QMenu{background-color:#292929;color:white;}"\
                                                 "QMenu::item:selected{background-color: #484848;}");
   //                    tool_button->setIcon(QIcon(":/icons/linestate.png"));
   //                    tool_button->addAction(a_map["DimLinear"]);
   //                    tool_button->addAction(a_map["DimLinearHor"]);
   //                    tool_button->addAction(a_map["DimLinearVer"]);
   //                    tool_button->addAction(a_map["DimRadial"]);
   //                    tool_button->addAction(a_map["DimDiametric"]);
   //                    tool_button->addAction(a_map["DimAngular"]);
   //                    tool_button->addAction(a_map["DimLeader"]);
                       tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                   "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                   "QToolButton{   \
                                       color:transparent;border:none;background:url(':/new/prefix1/1204.png')}"    \
                         "QToolButton:hover{background:url(':/new/prefix1/626.png');}"\
                         "QToolButton:pressed{background:url(':/new/prefix1/656.png');}"\
                         "QMenu{background:#292929;color:white}");

                       tool_button = Tab11->button21;
                       tool_button->setPopupMode(QToolButton::DelayedPopup);
                       tool_button->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
                       tool_button->setDefaultAction(a_map["DrawMText"]);
                       tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                   "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                   "QToolButton{   \
                                       color:transparent;border:none;background:url(':/new/prefix1/1205.png')}"    \
                         "QToolButton:hover{background:url(':/new/prefix1/627.png');}"\
                         "QToolButton:pressed{background:url(':/new/prefix1/655.png');}");


                       tool_button = Tab11->button22;
                       tool_button->setPopupMode(QToolButton::InstantPopup);
                       tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                       tool_button->setDefaultAction(a_map["AlignTop"]);
                       tool_button->setIconSize(QSize(30,33));
                       tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                   "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                   "QToolButton{   \
                                       color:transparent;border:none;background:url(':/new/prefix1/1213.png')}"    \
                         "QToolButton:hover{background:url(':/new/prefix1/628.png');}"\
                         "QToolButton:pressed{background:url(':/new/prefix1/657.png');}");

                       tool_button = Tab11->button23;
                       tool_button->setPopupMode(QToolButton::InstantPopup);
                       tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                       tool_button->setDefaultAction(a_map["Verticalcenter"]);
                       tool_button->setIconSize(QSize(36,27));
                       tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                   "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                   "QToolButton{   \
                                       color:transparent;border:none;background:url(':/new/prefix1/1212.png')}"    \
                         "QToolButton:hover{background:url(':/new/prefix1/629.png');}"\
                         "QToolButton:pressed{background:url(':/new/prefix1/658.png');}");

                       tool_button = Tab11->button24;
                       tool_button->setPopupMode(QToolButton::InstantPopup);
                       tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                       tool_button->setDefaultAction(a_map["AlignBottom"]);
                       tool_button->setIconSize(QSize(30,31));
                       tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                   "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                   "QToolButton{   \
                                       color:transparent;border:none;background:url(':/new/prefix1/1211.png')}"    \
                         "QToolButton:hover{background:url(':/new/prefix1/630.png');}"\
                         "QToolButton:pressed{background:url(':/new/prefix1/659.png');}");

                       tool_button = Tab11->button25;
                       tool_button->setPopupMode(QToolButton::InstantPopup);
                       tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                       tool_button->setDefaultAction(a_map["CenterAlignment"]);
                       tool_button->setIconSize(QSize(31,19));
                       tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                   "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                   "QToolButton{   \
                                       color:transparent;border:none;background:url(':/new/prefix1/1210.png')}"    \
                         "QToolButton:hover{background:url(':/new/prefix1/631.png');}"\
                         "QToolButton:pressed{background:url(':/new/prefix1/660.png');}");

                       tool_button = Tab11->button26;
                       tool_button->setPopupMode(QToolButton::InstantPopup);
                       tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                       tool_button->setDefaultAction(a_map["AlignLeft"]);
                       tool_button->setIconSize(QSize(33,30));
                       tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                   "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                   "QToolButton{   \
                                       color:transparent;border:none;background:url(':/new/prefix1/1209.png')}"    \
                         "QToolButton:hover{background:url(':/new/prefix1/632.png');}"\
                         "QToolButton:pressed{background:url(':/new/prefix1/661.png');}");

                       tool_button = Tab11->button27;
                       tool_button->setPopupMode(QToolButton::InstantPopup);
                       tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                       tool_button->setDefaultAction(a_map["HorizontalAlignment"]);
                       tool_button->setIconSize(QSize(27,36));
                       tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                   "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                   "QToolButton{   \
                                       color:transparent;border:none;background:url(':/new/prefix1/1208.png')no-repeat}"    \
                         "QToolButton:hover{background:url(':/new/prefix1/633.png')no-repeat;}"\
                         "QToolButton:pressed{background:url(':/new/prefix1/662.png')no-repeat;}");

                       tool_button = Tab11->button28;
                       tool_button->setPopupMode(QToolButton::InstantPopup);
                       tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                       tool_button->setDefaultAction(a_map["AlignRight"]);
                       tool_button->setIconSize(QSize(31,30));
                       tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                   "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                   "QToolButton{   \
                                       color:transparent;border:none;background:url(':/new/prefix1/1207.png')}"    \
                         "QToolButton:hover{background:url(':/new/prefix1/634.png');}"\
                         "QToolButton:pressed{background:url(':/new/prefix1/663.png');}");

    /*工艺界面*/
                    tool_button = Tab33->button1;
                    tool_button->setPopupMode(QToolButton::InstantPopup);
                    tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                    tool_button->setDefaultAction(a_map["YinXian"]);
                    tool_button->setIconSize(QSize(30,31));
                    tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                "QToolButton{   \
                                    color:transparent;border:none;background:url(':/technology/710.png')}"    \
                      "QToolButton:hover{background:url(':/technology/2141.png');}"\
                      "QToolButton:pressed{background:url(':/technology/2140.png');}");

                    tool_button = Tab33->button2;
                    tool_button->setPopupMode(QToolButton::InstantPopup);
                    tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                    tool_button->setDefaultAction(a_map["DistinguishBetweenInternalAndExternalModels"]);
                    tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                "QToolButton{   \
                                    color:transparent;border:none;background:url(':/technology/711.png')}"    \
                      "QToolButton:hover{background:url(':/technology/2142.png');}"\
                      "QToolButton:pressed{background:url(':/technology/2139.png');}");

                    tool_button = Tab33->button3;
                    tool_button->setPopupMode(QToolButton::InstantPopup);
                    tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                    tool_button->setDefaultAction(a_map["StartingPoint"]);
                    tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                "QToolButton{   \
                                    color:transparent;border:none;background:url(':/technology/712.png')}"    \
                      "QToolButton:hover{background:url(':/technology/2143.png');}"\
                      "QToolButton:pressed{background:url(':/technology/2138.png');}");

                    tool_button = Tab33->button4;
                    tool_button->setPopupMode(QToolButton::InstantPopup);
                    tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                    tool_button->setDefaultAction(a_map["Compensate"]);
                    tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                "QToolButton{   \
                                    color:transparent;border:none;background:url(':/technology/713.png')}"    \
                      "QToolButton:hover{background:url(':/technology/2144.png');}"\
                      "QToolButton:pressed{background:url(':/technology/2137.png');}");

                    tool_button = Tab33->button5;
                    tool_button->setPopupMode(QToolButton::InstantPopup);
                    tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                    tool_button->setDefaultAction(a_map["VaginalIncision"]);
                    tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                "QToolButton{   \
                                    color:transparent;border:none;background:url(':/technology/714.png')}"    \
                      "QToolButton:hover{background:url(':/technology/2145.png');}"\
                      "QToolButton:pressed{background:url(':/technology/2136.png');}");

                    tool_button = Tab33->button6;
                    tool_button->setPopupMode(QToolButton::InstantPopup);
                    tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                    tool_button->setDefaultAction(a_map["YangCut"]);
                    tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                "QToolButton{   \
                                    color:transparent;border:none;background:url(':/technology/715.png')}"    \
                      "QToolButton:hover{background:url(':/technology/2146.png');}"\
                      "QToolButton:pressed{background:url(':/technology/2135.png');}");

                    tool_button = Tab33->button7;
                    tool_button->setPopupMode(QToolButton::InstantPopup);
                    tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                    tool_button->setDefaultAction(a_map["Weilian"]);
                    tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                "QToolButton{   \
                                    color:transparent;border:none;background:url(':/technology/716.png')}"    \
                      "QToolButton:hover{background:url(':/technology/2147.png');}"\
                      "QToolButton:pressed{background:url(':/technology/2134.png');}");

                    tool_button = Tab33->button8;
                    tool_button->setPopupMode(QToolButton::InstantPopup);
                    tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                    tool_button->setDefaultAction(a_map["AutoWeilian"]);
                    tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                "QToolButton{   \
                                    color:transparent;border:none;background:url(':/technology/717.png')}"    \
                      "QToolButton:hover{background:url(':/technology/2148.png');}"\
                      "QToolButton:pressed{background:url(':/technology/2133.png');}");

                    tool_button = Tab33->button9;
                    tool_button->setPopupMode(QToolButton::InstantPopup);
                    tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                    tool_button->setDefaultAction(a_map["ExplodeWeilian"]);
                    tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                "QToolButton{   \
                                    color:transparent;border:none;background:url(':/technology/718.png')}"    \
                      "QToolButton:hover{background:url(':/technology/2149.png');}"\
                      "QToolButton:pressed{background:url(':/technology/2132.png');}");

                    tool_button = Tab33->button10;
                    tool_button->setPopupMode(QToolButton::InstantPopup);
                    tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                    tool_button->setDefaultAction(a_map["LengQue"]);
                    tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                "QToolButton{   \
                                    color:transparent;border:none;background:url(':/technology/719.png')}"    \
                      "QToolButton:hover{background:url(':/technology/2150.png');}"\
                      "QToolButton:pressed{background:url(':/technology/2131.png');}");

                    tool_button = Tab33->button11;
                    tool_button->setPopupMode(QToolButton::InstantPopup);
                    tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                    tool_button->setDefaultAction(a_map["CleanCoolingPoint"]);
                    tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                "QToolButton{   \
                                    color:transparent;border:none;background:url(':/technology/720.png')}"    \
                      "QToolButton:hover{background:url(':/technology/2151.png');}"\
                      "QToolButton:pressed{background:url(':/technology/2130.png');}");

                    tool_button = Tab33->button12;
                    tool_button->setPopupMode(QToolButton::InstantPopup);
                    tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                    tool_button->setDefaultAction(a_map["InfoDist1"]);
                    tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                "QToolButton{   \
                                    color:transparent;border:none;background:url(':/technology/721.png')}"    \
                      "QToolButton:hover{background:url(':/technology/2152.png');}"\
                      "QToolButton:pressed{background:url(':/technology/2129.png');}");

                    tool_button = Tab33->button13;
                    tool_button->setPopupMode(QToolButton::InstantPopup);
                    tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                    tool_button->setDefaultAction(a_map["LinearFlightCutting"]);
                    tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                "QToolButton{   \
                                    color:transparent;border:none;background:url(':/technology/722.png')}"    \
                      "QToolButton:hover{background:url(':/technology/2153.png');}"\
                      "QToolButton:pressed{background:url(':/technology/2128.png');}");

                    tool_button = Tab33->button14;
                    tool_button->setPopupMode(QToolButton::InstantPopup);
                    tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                    tool_button->setDefaultAction(a_map["CleanLeadWire"]);
                    tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                "QToolButton{   \
                                    color:transparent;border:none;background:url(':/technology/723.png')}"    \
                      "QToolButton:hover{background:url(':/technology/2154.png');}"\
                      "QToolButton:pressed{background:url(':/technology/2127.png');}");

                    tool_button = Tab33->button15;
                    tool_button->setPopupMode(QToolButton::InstantPopup);
                    tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                    tool_button->setDefaultAction(a_map["CleanWeilian"]);
                    tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                "QToolButton{   \
                                    color:transparent;border:none;background:url(':/technology/724.png')}"    \
                      "QToolButton:hover{background:url(':/technology/2155.png');}"\
                      "QToolButton:pressed{background:url(':/technology/2126.png');}");

                    tool_button = Tab33->button16;
                    tool_button->setPopupMode(QToolButton::InstantPopup);
                    tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                    tool_button->setDefaultAction(a_map["CancellationOfCompensation"]);
                    tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                "QToolButton{   \
                                    color:transparent;border:none;background:url(':/technology/725.png')}"    \
                      "QToolButton:hover{background:url(':/technology/2156.png');}"\
                      "QToolButton:pressed{background:url(':/technology/2125.png');}");

      /*数控*/
                    tool_button = Tab44->button1;
                    tool_button->setPopupMode(QToolButton::InstantPopup);
                    tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                    tool_button->setDefaultAction(a_map["ExportGCode"]);
                    tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                "QToolButton{   \
                                    color:transparent;border:none;background:url(':/digitalcontrol/744.png')}"    \
                      "QToolButton:hover{background:url(':/digitalcontrol/2242.png');}"\
                      "QToolButton:pressed{background:url(':/digitalcontrol/1683.png');}");
    /*排样*/
                    tool_button2 = Tab55->button1;
                    tool_button2->setPopupMode(QToolButton::InstantPopup);
                    tool_button2->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                    connect(tool_button2,SIGNAL(pressed()),this,SLOT(slotParts()));



//                    tool_button->addAction(a_map["FileOpenPreview"]);
//                    tool_button->addAction(a_map["AddFactory"]);
                    tool_button2->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                "QToolButton{   \
                                    color:transparent;border:none;background:url(':/distribution/2241.png')}"    \
                      "QToolButton:hover{background:url(':/distribution/2213.png');}"\
                      "QToolButton:pressed{background:url(':/distribution/2157.png');}"\
                      "QMenu{background:#292929;color:white}"\
                      "QMenu::item:selected{background-color: #484848;}");

                    tool_button = Tab55->button2;
                    tool_button->setPopupMode(QToolButton::InstantPopup);
                    tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                    tool_button->setDefaultAction(a_map["DeleteAllNestingParts"]);
                    tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                "QToolButton{   \
                                    color:transparent;border:none;background:url(':/distribution/1853.png')}"    \
                      "QToolButton:hover{background:url(':/distribution/2214.png');}"\
                      "QToolButton:pressed{background:url(':/distribution/2158.png');}"\
                      "QMenu{background:#292929;color:white}");

                    tool_button = Tab55->button3;
                    tool_button->setPopupMode(QToolButton::InstantPopup);
                    tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                    tool_button->setDefaultAction(a_map["SetBoard"]);
                    tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                "QToolButton{   \
                                    color:transparent;border:none;background:url(':/distribution/1854.png')}"    \
                      "QToolButton:hover{background:url(':/distribution/2215.png');}"\
                      "QToolButton:pressed{background:url(':/distribution/2159.png');}"\
                      "QMenu{background:#292929;color:white}");

                    tool_button = Tab55->button4;
                    tool_button->setPopupMode(QToolButton::InstantPopup);
                    tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                    tool_button->setDefaultAction(a_map["DeleteLayoutBoard"]);
                    tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                "QToolButton{   \
                                    color:transparent;border:none;background:url(':/distribution/1855.png')}"    \
                      "QToolButton:hover{background:url(':/distribution/2216.png');}"\
                      "QToolButton:pressed{background:url(':/distribution/2160.png');}"\
                      "QMenu{background:#292929;color:white}");

                    tool_button = Tab55->button5;
                    tool_button->setPopupMode(QToolButton::InstantPopup);
                    tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                    tool_button->setDefaultAction(a_map["Arrange"]);
                    tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                "QToolButton{   \
                                    color:transparent;border:none;background:url(':/distribution/1856.png')}"    \
                      "QToolButton:hover{background:url(':/distribution/2217.png');}"\
                      "QToolButton:pressed{background:url(':/distribution/2161.png');}"\
                      "QMenu{background:#292929;color:white}");

                    tool_button = Tab55->button6;
                    tool_button->setPopupMode(QToolButton::InstantPopup);
                    tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                    tool_button->setDefaultAction(a_map["ClearLayout"]);
                    tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                "QToolButton{   \
                                    color:transparent;border:none;background:url(':/distribution/1857.png')}"    \
                      "QToolButton:hover{background:url(':/distribution/2218.png');}"\
                      "QToolButton:pressed{background:url(':/distribution/2162.png');}"\
                      "QMenu{background:#292929;color:white}");

                    tool_button = Tab55->button7;
                    tool_button->setPopupMode(QToolButton::InstantPopup);
                    tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                    tool_button->addActions(select_actions);
                    tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                "QToolButton{   \
                                    color:transparent;border:none;background:url(':/distribution/1858.png')}"    \
                      "QToolButton:hover{background:url(':/distribution/2219.png');}"\
                      "QToolButton:pressed{background:url(':/distribution/2163.png');}"\
                      "QMenu{background:#292929;color:white}"\
                      "QMenu::item:selected{background-color: #484848;}");

                    tool_button = Tab55->button8;
                    tool_button->setPopupMode(QToolButton::InstantPopup);
                    tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);//xxx
                    tool_button->setDefaultAction(a_map["EditUndo"]);
                    tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                "QToolButton{   \
                                    color:transparent;border:none;background:url(':/distribution/1859.png')}"    \
                      "QToolButton:hover{background:url(':/distribution/2220.png');}"\
                      "QToolButton:pressed{background:url(':/distribution/2164.png');}"\
                      "QMenu{background:#292929;color:white}");

                    tool_button = Tab55->button9;
                    tool_button->setPopupMode(QToolButton::InstantPopup);
                    tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                    tool_button->setDefaultAction(a_map["EditRedo"]);
                    tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                "QToolButton{   \
                                    color:transparent;border:none;background:url(':/distribution/1860.png')}"    \
                      "QToolButton:hover{background:url(':/distribution/2221.png');}"\
                      "QToolButton:pressed{background:url(':/distribution/2165.png');}"\
                      "QMenu{background:#292929;color:white}");

                    tool_button = Tab55->button10;
                    tool_button->setPopupMode(QToolButton::InstantPopup);
                    tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                    tool_button->setDefaultAction(a_map["ModifyDeleteQuick1"]);
                    tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                "QToolButton{   \
                                    color:transparent;border:none;background:url(':/distribution/1861.png')}"    \
                      "QToolButton:hover{background:url(':/distribution/2222.png');}"\
                      "QToolButton:pressed{background:url(':/distribution/2166.png');}"\
                      "QMenu{background:#292929;color:white}");

                    tool_button = Tab55->button11;
                                 tool_button->setPopupMode(QToolButton::InstantPopup);
                                 tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                                 tool_button->setDefaultAction(a_map["EditCopy"]);
                                 tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                             "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                             "QToolButton{   \
                                                 color:transparent;border:none;background:url(':/distribution/1864.png')}"    \
                                   "QToolButton:hover{background:url(':/distribution/2225.png');}"\
                                   "QToolButton:pressed{background:url(':/distribution/2169.png');}"\
                                   "QMenu{background:#292929;color:white}");

                                 tool_button = Tab55->button12;
                                 tool_button->setPopupMode(QToolButton::InstantPopup);
                                 tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                                 tool_button->setDefaultAction(a_map["EditCut"]);
                                 tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                             "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                             "QToolButton{   \
                                                 color:transparent;border:none;background:url(':/distribution/1863.png')}"    \
                                   "QToolButton:hover{background:url(':/distribution/2224.png');}"\
                                   "QToolButton:pressed{background:url(':/distribution/2168.png');}"\
                                   "QMenu{background:#292929;color:white}");

                                 tool_button = Tab55->button13;
                                 tool_button->setPopupMode(QToolButton::InstantPopup);
                                 tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                                 tool_button->setDefaultAction(a_map["EditPaste"]);
                                 tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                             "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                             "QToolButton{   \
                                                 color:transparent;border:none;background:url(':/distribution/1862.png')}"    \
                                   "QToolButton:hover{background:url(':/distribution/2223.png');}"\
                                   "QToolButton:pressed{background:url(':/distribution/2167.png');}"\
                                   "QMenu{background:#292929;color:white}");

                                 tool_button = Tab55->button14;
                                 tool_button->setPopupMode(QToolButton::InstantPopup);
                                 tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                                 tool_button->setDefaultAction(a_map["DefineGroups"]);
                                 tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                             "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                             "QToolButton{   \
                                                 color:transparent;border:none;background:url(':/distribution/1865.png')}"    \
                                   "QToolButton:hover{background:url(':/distribution/2226.png');}"\
                                   "QToolButton:pressed{background:url(':/distribution/2170.png');}"\
                                   "QMenu{background:#292929;color:white}");

                                 tool_button = Tab55->button15;
                                 tool_button->setPopupMode(QToolButton::InstantPopup);
                                 tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                                 tool_button->setDefaultAction(a_map["BreakUpSelectedGroups"]);
                                 tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                             "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                             "QToolButton{   \
                                                 color:transparent;border:none;background:url(':/distribution/1866.png')}"    \
                                   "QToolButton:hover{background:url(':/distribution/2227.png');}"\
                                   "QToolButton:pressed{background:url(':/distribution/2171.png');}"\
                                   "QMenu{background:#292929;color:white}");

                                 tool_button = Tab55->button16;
                                 tool_button->setPopupMode(QToolButton::InstantPopup);
                                 tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                                 tool_button->setDefaultAction(a_map["BlocksExplode"]);
                                 tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                             "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                             "QToolButton{   \
                                                 color:transparent;border:none;background:url(':/distribution/1867.png')}"    \
                                   "QToolButton:hover{background:url(':/distribution/2228.png');}"\
                                   "QToolButton:pressed{background:url(':/distribution/2172.png');}"\
                                   "QMenu{background:#292929;color:white}");

                                 tool_button = Tab55->button17;
                                 tool_button->setPopupMode(QToolButton::InstantPopup);
                                 tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                                 tool_button->setDefaultAction(a_map["MatrixPermutation"]);
                                 tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                             "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                             "QToolButton{   \
                                                 color:transparent;border:none;background:url(':/distribution/1868.png')}"    \
                                   "QToolButton:hover{background:url(':/distribution/2229.png');}"\
                                   "QToolButton:pressed{background:url(':/distribution/2173.png');}"\
                                   "QMenu{background:#292929;color:white}");

                                 tool_button = Tab55->button18;
                                 tool_button->setPopupMode(QToolButton::InstantPopup);
                                 tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                                 tool_button->setDefaultAction(a_map["InteractiveArrangement"]);
                                 tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                             "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                             "QToolButton{   \
                                                 color:transparent;border:none;background:url(':/distribution/1869.png')}"    \
                                   "QToolButton:hover{background:url(':/distribution/2230.png');}"\
                                   "QToolButton:pressed{background:url(':/distribution/2174.png');}"\
                                   "QMenu{background:#292929;color:white}");

                                 tool_button = Tab55->button19;
                                 tool_button->setPopupMode(QToolButton::InstantPopup);
                                 tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                                 tool_button->setDefaultAction(a_map["FullOf"]);
                                 tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                             "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                             "QToolButton{   \
                                                 color:transparent;border:none;background:url(':/distribution/1870.png')}"    \
                                   "QToolButton:hover{background:url(':/distribution/2231.png');}"\
                                   "QToolButton:pressed{background:url(':/distribution/2175.png');}"\
                                   "QMenu{background:#292929;color:white}");

                                 tool_button = Tab55->button20;
                                 tool_button->setPopupMode(QToolButton::InstantPopup);
                                 tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                                 tool_button->setDefaultAction(a_map["Bridging"]);
                                 tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                             "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                             "QToolButton{   \
                                                 color:transparent;border:none;background:url(':/distribution/1871.png')}"    \
                                   "QToolButton:hover{background:url(':/distribution/2232.png');}"\
                                   "QToolButton:pressed{background:url(':/distribution/2176.png');}"\
                                   "QMenu{background:#292929;color:white}");

                                 tool_button = Tab55->button21;
                                 tool_button->setPopupMode(QToolButton::InstantPopup);
                                 tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                                 tool_button->setDefaultAction(a_map["CommonEdge"]);
                                 tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                             "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                             "QToolButton{   \
                                                 color:transparent;border:none;background:url(':/distribution/1872.png')}"    \
                                   "QToolButton:hover{background:url(':/distribution/2233.png');}"\
                                   "QToolButton:pressed{background:url(':/distribution/2177.png');}"\
                                   "QMenu{background:#292929;color:white}");

                                 tool_button = Tab55->button22;
                                 tool_button->setPopupMode(QToolButton::InstantPopup);
                                 tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                                 tool_button->setDefaultAction(a_map["ModifyCut"]);
                                 tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                             "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                             "QToolButton{   \
                                                 color:transparent;border:none;background:url(':/distribution/1873.png')}"    \
                                   "QToolButton:hover{background:url(':/distribution/2234.png');}"\
                                   "QToolButton:pressed{background:url(':/distribution/2178.png');}"\
                                   "QMenu{background:#292929;color:white}");

                                 tool_button = Tab55->button23;
                                 tool_button->setPopupMode(QToolButton::InstantPopup);
                                 tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                                 tool_button->setDefaultAction(a_map["RemoveDuplicateLines"]);
                                 tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                             "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                             "QToolButton{   \
                                                 color:transparent;border:none;background:url(':/distribution/1874.png')}"    \
                                   "QToolButton:hover{background:url(':/distribution/2235.png');}"\
                                   "QToolButton:pressed{background:url(':/distribution/2179.png');}"\
                                   "QMenu{background:#292929;color:white}");

                                 tool_button = Tab55->button24;
                                 tool_button->setPopupMode(QToolButton::InstantPopup);
                                 tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                                 tool_button->setDefaultAction(a_map["MergeConnectors"]);
                                 tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                             "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                             "QToolButton{   \
                                                 color:transparent;border:none;background:url(':/distribution/1875.png')}"    \
                                   "QToolButton:hover{background:url(':/distribution/2236.png');}"\
                                   "QToolButton:pressed{background:url(':/distribution/2180.png');}"\
                                   "QMenu{background:#292929;color:white}");

                                 tool_button = Tab55->button25;
                                 tool_button->setPopupMode(QToolButton::InstantPopup);//xxxx
                                 tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                                 tool_button->addActions(sort_actions);
                                 tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                             "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                             "QToolButton{   \
                                                 color:transparent;border:none;background:url(':/distribution/1876.png')}"    \
                                   "QToolButton:hover{background:url(':/distribution/2237.png');}"\
                                   "QToolButton:pressed{background:url(':/distribution/2181.png');}"\
                                   "QMenu{background:#292929;color:white}"\
                                   "QMenu::item:selected{background-color: #484848;}");

                                 tool_button = Tab55->button26;
                                 tool_button->setPopupMode(QToolButton::InstantPopup);
                                 tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                                 tool_button->setDefaultAction(a_map["PreventSortingFromChangingDirection"]);
                                 tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                             "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                             "QToolButton{   \
                                                 color:transparent;border:none;background:url(':/distribution/1879.png')}"    \
                                   "QToolButton:hover{background:url(':/distribution/2238.png');}"\
                                   "QToolButton:pressed{background:url(':/distribution/2182.png');}"\
                                   "QMenu{background:#292929;color:white}");

                                 tool_button = Tab55->button27;
                                 tool_button->setPopupMode(QToolButton::InstantPopup);
                                 tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                                 tool_button->setDefaultAction(a_map["InnerOuterModelOfSortingTimeZone"]);
                                 tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                             "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                             "QToolButton{   \
                                                 color:transparent;border:none;background:url(':/distribution/1878.png')}"    \
                                   "QToolButton:hover{background:url(':/distribution/2239.png');}"\
                                   "QToolButton:pressed{background:url(':/distribution/2183.png');}"\
                                   "QMenu{background:#292929;color:white}");

                                 tool_button = Tab55->button28;
                                 tool_button->setPopupMode(QToolButton::InstantPopup);
                                 tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                                 tool_button->setDefaultAction(a_map["TheOutermostLayerIsIncised"]);
                                 tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                             "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                             "QToolButton{   \
                                                 color:transparent;border:none;background:url(':/distribution/1877.png')}"    \
                                   "QToolButton:hover{background:url(':/distribution/2240.png');}"\
                                   "QToolButton:pressed{background:url(':/distribution/2184.png');}"\
                                   "QMenu{background:#292929;color:white}");




                    /*视图*/
                                       tool_button = Tab66->button1;
                                       tool_button->setPopupMode(QToolButton::InstantPopup);
                                       tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                                       tool_button->setDefaultAction(a_map["Fullscreen"]);
                                       tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                                   "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                                   "QToolButton{   \
                                                       color:transparent;border:none;background:url(':/view/788.png')}"    \
                                         "QToolButton:hover{background:url(':/view/2265.png');}"\
                                         "QToolButton:pressed{background:url(':/view/2266.png');}"\
                                         "QMenu{background:#292929;color:white}");

                   //                    tool_button = Tab66->button2;
                   //                    tool_button->setPopupMode(QToolButton::InstantPopup);
                   //                    tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                   //                    tool_button->setDefaultAction(a_map["ViewStatusBar"]);
                   //                    tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                   //                                "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                   //                                "QToolButton{   \
                   //                                    color:transparent;border:none;background:url(':/view/789.png')}"    \
                   //                      "QToolButton:hover{background:url(':/view/2261.png');}"\
                   //                      "QToolButton:pressed{background:url(':/view/2267.png');}"\
                   //                      "QMenu{background:#292929;color:white}");

                                       tool_button = Tab66->button3;
                                       tool_button->setPopupMode(QToolButton::InstantPopup);
                                       tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                                       tool_button->setDefaultAction(a_map["ViewGrid"]);
                                       tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                                   "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                                   "QToolButton{   \
                                                       color:transparent;border:none;background:url(':/view/790.png')}"    \
                                         "QToolButton:hover{background:url(':/view/2262.png');}"\
                                         "QToolButton:pressed{background:url(':/view/2268.png');}"\
                                         "QMenu{background:#292929;color:white}");

                                       tool_button = Tab66->button4;
                                       tool_button->setPopupMode(QToolButton::InstantPopup);
                                       tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                                       tool_button->setDefaultAction(a_map["ViewDraft"]);
                                       tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                                   "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                                   "QToolButton{   \
                                                       color:transparent;border:none;background:url(':/view/791.png')}"    \
                                         "QToolButton:hover{background:url(':/view/2263.png');}"\
                                         "QToolButton:pressed{background:url(':/view/2269.png');}"\
                                         "QMenu{background:#292929;color:white}");

                                       tool_button = Tab66->button5;
                                       tool_button->setPopupMode(QToolButton::InstantPopup);
                                       tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                                       tool_button->setDefaultAction(a_map["ZoomRedraw"]);
                                       tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                                   "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                                   "QToolButton{   \
                                                       color:transparent;border:none;background:url(':/view/2277.png')}"    \
                                         "QToolButton:hover{background:url(':/view/2260.png');}"\
                                         "QToolButton:pressed{background:url(':/view/2270.png');}"\
                                         "QMenu{background:#292929;color:white}");

                                       tool_button = Tab66->button6;
                                       tool_button->setPopupMode(QToolButton::InstantPopup);
                                       tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                                       tool_button->setDefaultAction(a_map["ZoomWindow"]);
                                       tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                                   "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                                   "QToolButton{   \
                                                       color:transparent;border:none;background:url(':/view/2278.png')}"    \
                                         "QToolButton:hover{background:url(':/view/2259.png');}"\
                                         "QToolButton:pressed{background:url(':/view/2271.png');}"\
                                         "QMenu{background:#292929;color:white}");

                                       tool_button = Tab66->button7;
                                       tool_button->setPopupMode(QToolButton::InstantPopup);
                                       tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                                       tool_button->setDefaultAction(a_map["ZoomPrevious"]);
                                       tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                                   "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                                   "QToolButton{   \
                                                       color:transparent;border:none;background:url(':/view/2279.png')}"    \
                                         "QToolButton:hover{background:url(':/view/2258.png');}"\
                                         "QToolButton:pressed{background:url(':/view/2272.png');}"\
                                         "QMenu{background:#292929;color:white}");

                                       tool_button = Tab66->button8;
                                       tool_button->setPopupMode(QToolButton::InstantPopup);
                                       tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                                       tool_button->setDefaultAction(a_map["ZoomIn"]);
                                       tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                                   "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                                   "QToolButton{   \
                                                       color:transparent;border:none;background:url(':/view/793.png')}"    \
                                         "QToolButton:hover{background:url(':/view/2255.png');}"\
                                         "QToolButton:pressed{background:url(':/view/2273.png');}"\
                                         "QMenu{background:#292929;color:white}");

                                       tool_button = Tab66->button9;
                                       tool_button->setPopupMode(QToolButton::InstantPopup);
                                       tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                                       tool_button->setDefaultAction(a_map["ZoomAuto"]);
                                       tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                                   "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                                   "QToolButton{   \
                                                       color:transparent;border:none;background:url(':/view/794.png')}"    \
                                         "QToolButton:hover{background:url(':/view/2256.png');}"\
                                         "QToolButton:pressed{background:url(':/view/2274.png');}"\
                                         "QMenu{background:#292929;color:white}");

                                       tool_button = Tab66->button10;
                                       tool_button->setPopupMode(QToolButton::InstantPopup);
                                       tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                                       tool_button->setDefaultAction(a_map["ZoomPan"]);
                                       tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                                   "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                                   "QToolButton{   \
                                                       color:transparent;border:none;background:url(':/view/795.png')}"    \
                                         "QToolButton:hover{background:url(':/view/2257.png');}"\
                                         "QToolButton:pressed{background:url(':/view/2275.png');}"\
                                         "QMenu{background:#292929;color:white}");

                                       tool_button = Tab66->button11;
                                       tool_button->setPopupMode(QToolButton::InstantPopup);
                                       tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                                       tool_button->setDefaultAction(a_map["ZoomOut"]);
                                       tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                                   "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                                   "QToolButton{   \
                                                       color:transparent;border:none;background:url(':/view/792.png')}"    \
                                         "QToolButton:hover{background:url(':/view/2254.png');}"\
                                         "QToolButton:pressed{background:url(':/view/2276.png');}"\
                                         "QMenu{background:#292929;color:white}");
                                       /*帮助*/
                                                           tool_button = Tab77->button1;
                                                           tool_button->setPopupMode(QToolButton::InstantPopup);
                                                           tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                                                           tool_button->setDefaultAction(a_map["settings"]);
                                                           tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                                                       "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                                                       "QToolButton{   \
                                                                           color:transparent;border:none;background:url(':/view/564.png')}"    \
                                                             "QToolButton:hover{background:url(':/view/1926.png');}"\
                                                             "QToolButton:pressed{background:url(':/view/568.png');}"\
                                                             "QMenu{background:#292929;color:white}");


                                                           QAction* wiki_link = new QAction(QC_ApplicationWindow::tr("Online"), main_window);//help
                                                           connect(wiki_link, SIGNAL(triggered()), main_window, SLOT(invokeLinkList()));
                                                           QAction* help_about = new QAction( QC_ApplicationWindow::tr("About"), main_window);
                                                           connect(help_about, SIGNAL(triggered()), main_window, SLOT(showAboutWindow()));
                                                           QAction* license = new QAction(QObject::tr("License"), main_window);
                                                           connect(license, SIGNAL(triggered()), main_window, SLOT(invokeLicenseWindow()));

                                                           tool_button = Tab77->button2;
                                                           tool_button->setPopupMode(QToolButton::InstantPopup);
                                                           tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                                                           tool_button->setDefaultAction(wiki_link);
                                                           tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                                                       "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                                                       "QToolButton{   \
                                                                           color:transparent;border:none;background:url(':/view/565.png')}"    \
                                                             "QToolButton:hover{background:url(':/view/1929.png');}"\
                                                             "QToolButton:pressed{background:url(':/view/569.png');}"\
                                                             "QMenu{background:#292929;color:white}");

                                                           tool_button = Tab77->button3;
                                                           tool_button->setPopupMode(QToolButton::InstantPopup);
                                                           tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                                                           tool_button->setDefaultAction(help_about);
                                                           tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                                                       "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                                                       "QToolButton{   \
                                                                           color:transparent;border:none;background:url(':/view/566.png')}"    \
                                                             "QToolButton:hover{background:url(':/view/1927.png');}"\
                                                             "QToolButton:pressed{background:url(':/view/570.png');}"\
                                                             "QMenu{background:#292929;color:white}");

                                                           tool_button = Tab77->button4;
                                                           tool_button->setPopupMode(QToolButton::InstantPopup);
                                                           tool_button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
                                                           tool_button->setDefaultAction(license);
                                                           tool_button->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                                                                       "QToolTip{border: 0px solid;background:rgb(48,48,48);color:#0ff}"\
                                                                       "QToolButton{   \
                                                                           color:transparent;border:none;background:url(':/view/567.png')}"    \
                                                             "QToolButton:hover{background:url(':/view/1928.png');}"\
                                                             "QToolButton:pressed{background:url(':/view/571.png');}"\
                                                             "QMenu{background:#292929;color:white}");
                                           QWidget *Tab2 = new QWidget();




    tabToolbar->addTab(Tab2, "常用");
    tabToolbar->addTab(Tab11, "绘图");
    tabToolbar->addTab(Tab33, "工艺");
    tabToolbar->addTab(Tab44, "数控");
    tabToolbar->addTab(Tab55, "排样");
    tabToolbar->addTab(Tab66, "视图");
    tabToolbar->addTab(Tab77, " 帮助 ");
    tab_toolbar->setFixedHeight(126);
//    tabToolbar->setFixedHeight(120);
    tabToolbar->setStyleSheet("QTabBar::tab { height: 25px; width:55px;color: white; background-color: #4d4d4d;}"
                                     "QTabBar::tab:selected { background-color: rgb(56,56,56); }"
                              "QTabWidget::pane {height: 95px;border-top: 1px solid #000000;\
                              border-bottom: 6px solid rgb(108,111,114);\
                              border-left: 1px solid #000;background-color:rgb(48,48,48)}"\
                               "QTabBar{color:white}");
    tabToolbar->setCurrentIndex(1);
    tab_toolbar->addWidget(tabToolbar);
    tab_toolbar->setMovable(false);
    tab_toolbar->setSizePolicy(toolBarPolicy);
    tab_toolbar->setObjectName("tab_toolbar");
    tab_toolbar->addWidget(fileWindow1);


    // <[~ Dock Areas Toolbar ~]>

//    QToolBar* dockareas_toolbar = new QToolBar(main_window);
//    dockareas_toolbar->setWindowTitle(QC_ApplicationWindow::tr("Dock Areas"));
//    dockareas_toolbar->setSizePolicy(toolBarPolicy);
//    dockareas_toolbar->setObjectName("dockareas_toolbar");
//    dockareas_toolbar->addAction(a_map["LeftDockAreaToggle"]);
//    dockareas_toolbar->addAction(a_map["RightDockAreaToggle"]);
//    dockareas_toolbar->addAction(a_map["TopDockAreaToggle"]);
//    dockareas_toolbar->addAction(a_map["BottomDockAreaToggle"]);
//    dockareas_toolbar->addAction(a_map["FloatingDockwidgetsToggle"]);
//    dockareas_toolbar->setMovable(false);
    // <[~ Creators ~]>

//    auto creators_toolbar = new QToolBar(main_window);
//    creators_toolbar->setWindowTitle(QObject::tr("Creators"));
//    creators_toolbar->setObjectName("creators_toolbar");
//    creators_toolbar->addAction(a_map["InvokeMenuCreator"]);
//    creators_toolbar->addAction(a_map["InvokeToolbarCreator"]);
//    creators_toolbar->setMovable(false);

    //main_window->addToolBar(Qt::TopToolBarArea, edit_toolbar);
    // <[~ Toolbars Layout~]>
  /*  main_window->addToolBar(Qt::TopToolBarArea, file_toolbar);
    main_window->addToolBar(Qt::TopToolBarArea, edit_toolbar);
    main_window->addToolBar(Qt::TopToolBarArea, view_toolbar);

    main_window->addToolBarBreak();
    main_window->addToolBar(Qt::TopToolBarArea, processsetup_toolbar);
    main_window->addToolBar(Qt::TopToolBarArea, array_toolbar);
    main_window->addToolBar(Qt::TopToolBarArea, group_toolbar);
    main_window->addToolBar(Qt::TopToolBarArea, bridging_toolbar);
    main_window->addToolBarBreak();
    main_window->addToolBar(Qt::TopToolBarArea, pen_toolbar);
    main_window->addToolBar(Qt::TopToolBarArea, technology_toolbar);
    main_window->addToolBar(Qt::TopToolBarArea, clean_toolbar);
    main_window->addToolBar(Qt::TopToolBarArea, sortmanually_toolbar);
    main_window->addToolBar(Qt::TopToolBarArea, layout_toolbar);*/


//    main_window->addToolBar(Qt::TopToolBarArea, view_toolbar);
//    main_window->addToolBarBreak();
//    file_toolbar->setStyleSheet("QToolTip{background-color:black}");
//    main_window->addToolBar(Qt::TopToolBarArea, file_toolbar);
//    main_window->addToolBarBreak();
//    main_window->addToolBar(Qt::TopToolBarArea, sortmanually_toolbar);

    //main_window->addToolBarBreak(Qt::TopToolBarArea);
   // main_window->addToolBar(Qt::TopToolBarArea, edit_toolbar);

//    QMainWindow *titleWindow = new QMainWindow;
//    titleWindow->setStyleSheet("QToolBar{border-color: rgb(0, 0, 0);border:0px solid;}");
//    titleWindow->addToolBar(Qt::TopToolBarArea, title_toolbar);
//    titleWindow->addToolBar(Qt::TopToolBarArea, middle_toolbar);
//    titleWindow->addToolBar(Qt::TopToolBarArea, button_toolbar);

//    main_window->addDockWidget(titleWindow);
//    main_window->addToolBar(Qt::TopToolBarArea, title_toolbar);
//    main_window->addToolBar(Qt::TopToolBarArea, middle_toolbar);
//    main_window->addToolBar(Qt::TopToolBarArea, button_toolbar);
  //  main_window->addToolBarBreak(Qt::TopToolBarArea);
//    main_window->setStyleSheet("background-color: #4d4d4d");
//    main_window->setStyleSheet("background-color: red");
//    main_window->addToolBar(Qt::TopToolBarArea, file_toolbar);

 //   main_window->setStyleSheet("body{background-color:#4d4d4d;}");
  //  main_window->setStyle("background-color:#4d4d4d");
    tab_toolbar->setStyleSheet("QToolBar#tab_toolbar{background-color:#4d4d4d;margin-left:0px;margin-top:0px;}");
    main_window->addToolBar(Qt::TopToolBarArea, tab_toolbar);

    tabToolbar->setFixedHeight(150);//标题和主题中间
    options_toolbar->setStyleSheet("background-color:#4d4d4d;margin:0px;border-top: 1px solid #4d4d4d;");
//    options_toolbar->setFixedWidth(50);
    options_toolbar->setMinimumWidth(50);
    order_toolbar->setStyleSheet("background-color:#4d4d4d;");
    snap_toolbar->setStyleSheet("QG_SnapToolBar#snap_toolbar{background-color:#303030;margin-left:0px;margin-top:0px;border: 1px solid black;}"\
                                "QG_SnapToolBar::separator{background-color:black;width:1px}");
    snap_toolbar->setFixedHeight(36);
    main_window->addToolBar(Qt::LeftToolBarArea, options_toolbar);
    main_window->addToolBar(Qt::LeftToolBarArea, order_toolbar);
    main_window->addToolBar(Qt::BottomToolBarArea, snap_toolbar);


//    main_window->addToolBar(Qt::BottomToolBarArea, dockareas_toolbar);
//    main_window->addToolBar(Qt::BottomToolBarArea, creators_toolbar);
}


//QStatusBar* status_bar = main_window->statusBar();

//void LC_WidgetFactory::mouseMoveEvent(QMouseEvent * event){
//  qDebug() << "ssaa";
//}

//void LC_WidgetFactory::mouseDoubleClickEvent(QMouseEvent *e){
//    qDebug() << "wwwwwwwwwwwwww";
//}

void LC_WidgetFactory::createCADToolbars()
{
    QSizePolicy toolBarPolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    QToolBar* line_toolbar = new QToolBar(QC_ApplicationWindow::tr("Line"), main_window);
    line_toolbar->setSizePolicy(toolBarPolicy);
    line_toolbar->setObjectName("line_toolbar");
    line_toolbar->addActions(line_actions1);
    line_toolbar->hide();
    line_toolbar->setMovable(false);

    QToolBar* circle_toolbar = new QToolBar(QC_ApplicationWindow::tr("Circle"), main_window);
    circle_toolbar->setSizePolicy(toolBarPolicy);
    circle_toolbar->setObjectName ("circle_toolbar");
    circle_toolbar->addActions(circle_actions1);
    circle_toolbar->hide();
    circle_toolbar->setMovable(false);

    QToolBar* curve_toolbar = new QToolBar(QC_ApplicationWindow::tr("Curve"), main_window);
    curve_toolbar->setSizePolicy(toolBarPolicy);
    curve_toolbar->setObjectName("curve_toolbar");
    curve_toolbar->addActions(curve_actions1);
    curve_toolbar->hide();
    curve_toolbar->setMovable(false);

    QToolBar* ellipse_toolbar = new QToolBar(QC_ApplicationWindow::tr("Ellipse"), main_window);
    ellipse_toolbar->setSizePolicy(toolBarPolicy);
    ellipse_toolbar->setObjectName("ellipse_toolbar");
    ellipse_toolbar->addActions(ellipse_actions);
    ellipse_toolbar->hide();
    ellipse_toolbar->setMovable(false);

    QToolBar* polyline_toolbar = new QToolBar(QC_ApplicationWindow::tr("Polyline"), main_window);
    polyline_toolbar->setSizePolicy(toolBarPolicy);
    polyline_toolbar->setObjectName("polyline_toolbar");
    polyline_toolbar->addActions(polyline_actions1);
    polyline_toolbar->addSeparator();
    polyline_toolbar->addActions(polyline_actions2);
    polyline_toolbar->addSeparator();
    polyline_toolbar->addActions(polyline_actions3);
    polyline_toolbar->addSeparator();
    polyline_toolbar->hide();
    polyline_toolbar->setMovable(false);

    QToolBar* select_toolbar = new QToolBar(QC_ApplicationWindow::tr("Select"), main_window);
    select_toolbar->setSizePolicy(toolBarPolicy);
    select_toolbar->setObjectName("select_toolbar");
    select_toolbar->addActions(select_actions1);
    select_toolbar->addSeparator();
    select_toolbar->addActions(select_actions2);
    select_toolbar->addSeparator();
    select_toolbar->addActions(select_actions3);
    select_toolbar->addSeparator();
    select_toolbar->addActions(select_actions4);
    select_toolbar->hide();
    select_toolbar->setMovable(false);
    QToolBar* dimension_toolbar = new QToolBar(QC_ApplicationWindow::tr("Dimension"), main_window);
    dimension_toolbar->setSizePolicy(toolBarPolicy);
    dimension_toolbar->setObjectName("dimension_toolbar");
    dimension_toolbar->addActions(dimension_actions);
    dimension_toolbar->hide();
    dimension_toolbar->setMovable(false);

    QToolBar* modify_toolbar = new QToolBar(QC_ApplicationWindow::tr("Modify"), main_window);
    modify_toolbar->setSizePolicy(toolBarPolicy);
    modify_toolbar->setObjectName("modify_toolbar");
    modify_toolbar->addActions(modify_actions);
    modify_toolbar->hide();
    modify_toolbar->setMovable(false);

    QToolBar* info_toolbar = new QToolBar(QC_ApplicationWindow::tr("Info"), main_window);
    info_toolbar->setSizePolicy(toolBarPolicy);
    info_toolbar->setObjectName("info_toolbar");
    info_toolbar->addActions(info_actions);
    info_toolbar->hide();
    info_toolbar->setMovable(false);
    main_window->addToolBar(Qt::BottomToolBarArea, line_toolbar);
    main_window->addToolBar(Qt::BottomToolBarArea, circle_toolbar);
    main_window->addToolBar(Qt::BottomToolBarArea, curve_toolbar);
    main_window->addToolBar(Qt::BottomToolBarArea, ellipse_toolbar);
    main_window->addToolBar(Qt::BottomToolBarArea, polyline_toolbar);
    main_window->addToolBar(Qt::BottomToolBarArea, dimension_toolbar);
    main_window->addToolBar(Qt::BottomToolBarArea, modify_toolbar);
    main_window->addToolBar(Qt::BottomToolBarArea, info_toolbar);
    main_window->addToolBar(Qt::BottomToolBarArea, select_toolbar);
}

QToolBar* LC_WidgetFactory::createCategoriesToolbar()
{
    QSizePolicy toolBarPolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    QMenu *button_menu = new QMenu();

    part_widget = new QToolBar(QC_ApplicationWindow::tr("www"), main_window);
    part_widget->setSizePolicy(toolBarPolicy);
    part_widget->setObjectName("part_widget");
    part_widget->setMovable(false);
    partWidget = new ss_PartWidget("");
  //  part_widget->setFixedSize(325,850);
    part_widget->setFixedSize(0,0);
    connect(partWidget->button,SIGNAL(pressed()),this,SLOT(slotCloseParts()));


    QWidget* pTitleWidget = partWidget ->titleBarWidget();
    QWidget* pWidget = new QWidget;
    partWidget ->setTitleBarWidget(pWidget );
    delete pTitleWidget ;

    part_widget->setStyleSheet("QToolBar#part_widget{background-color:#4d4d4d;margin-top:0px;border-top: 0.5px solid #4d4d4d;border-bottom:1px solid #707070;}");

    part_widget->addWidget(partWidget);




    QToolBar* categories_toolbar = new QToolBar(QC_ApplicationWindow::tr("Categories"), main_window);
    categories_toolbar->setSizePolicy(toolBarPolicy);
    categories_toolbar->setObjectName("categories_toolbar");

    QToolButton* tool_button;

    tool_button = new QToolButton;
    tool_button->setFixedSize(50,45);
    tool_button->setPopupMode(QToolButton::InstantPopup);
    tool_button->setStyleSheet("QToolButton::menu-indicator{image:none;}"\
                               "QToolButton{background:url(':/icons/201.png');border:none;}"\
                               "QToolButton::pressed{background:url(':/icons/101.png')}"\
                               "QToolButton::hover{background:url(':/icons/301.png')}"\
                               "QMenu{background:#292929;color:white}");
    categories_toolbar->addWidget(tool_button);
//    tool_button->addActions(line_actions);
    button_menu = new QMenu();
    button_menu->addActions(line_actions1);
    button_menu->addSeparator();
    button_menu->addActions(line_actions2);
    button_menu->addSeparator();
    button_menu->addActions(line_actions3);
    CustomIconStyle* pStyle = new CustomIconStyle;
    button_menu->setStyle(pStyle);
    button_menu->setStyleSheet("QMenu{background:#292929;color:white}"\
                       "QMenu::separator{background-color:#626262;height:1px}"\
                               "QMenu::item:selected{background-color: #484848;}");
    tool_button->setMenu(button_menu);

    tool_button = new QToolButton;
    tool_button->setFixedSize(50,45);
    tool_button->setPopupMode(QToolButton::InstantPopup);
    tool_button->setStyleSheet("QToolButton::menu-indicator{image:none;}"\
                               "QToolButton{background:url(':/icons/202.png');border:0px;}"\
                               "QToolButton::pressed{background:url(':/icons/102.png')}"\
                               "QToolButton::hover{background:url(':/icons/302.png')}"\
                               "QMenu{background:#292929;color:white}");
    categories_toolbar->addWidget(tool_button);
    button_menu = new QMenu();
    button_menu->addActions(circle_actions1);
    button_menu->addSeparator();
    button_menu->addActions(circle_actions2);
    button_menu->setStyle(pStyle);
    button_menu->setStyleSheet("QMenu{background:#292929;color:white}"\
                       "QMenu::separator{background-color:#626262;height:1px}"\
                               "QMenu::item:selected{background-color: #484848;}");
    tool_button->setMenu(button_menu);
//    tool_button->addActions(circle_actions);

    tool_button = new QToolButton;
    tool_button->setFixedSize(50,45);
    tool_button->setPopupMode(QToolButton::InstantPopup);
    tool_button->setStyleSheet("QToolButton::menu-indicator{image:none;}"\
                               "QToolButton{background:url(':/icons/203.png');border:0px;}"\
                               "QToolButton::pressed{background:url(':/icons/103.png')}"\
                               "QToolButton::hover{background:url(':/icons/303.png')}"\
                               "QMenu{background:#292929;color:white}");

    categories_toolbar->addWidget(tool_button);
//    tool_button->addActions(curve_actions);
    button_menu = new QMenu();
    button_menu->addActions(curve_actions1);
    button_menu->addSeparator();
    button_menu->addActions(curve_actions2);
    button_menu->addSeparator();
    button_menu->addActions(curve_actions3);
    button_menu->addSeparator();
    button_menu->addActions(curve_actions4);
    button_menu->setStyle(pStyle);
    button_menu->setStyleSheet("QMenu{background:#292929;color:white}"\
                       "QMenu::separator{background-color:#626262;height:1px}"\
                               "QMenu::item:selected{background-color: #484848;}");
    tool_button->setMenu(button_menu);

    tool_button = new QToolButton;
    tool_button->setFixedSize(50,45);
    tool_button->setPopupMode(QToolButton::InstantPopup);
    tool_button->setStyleSheet("QToolButton::menu-indicator{image:none;}"\
                               "QToolButton{background:url(':/icons/204.png');border:0px;}"\
                               "QToolButton::pressed{background:url(':/icons/104.png')}"\
                               "QToolButton::hover{background:url(':/icons/304.png')}"\
                               "QMenu{background:#292929;color:white}");
    categories_toolbar->addWidget(tool_button);
//    tool_button->addActions(ellipse_actions);
    button_menu = new QMenu();
    button_menu->addActions(ellipse_actions);
    button_menu->setStyle(pStyle);
    button_menu->setStyleSheet("QMenu{background:#292929;color:white}"\
                       "QMenu::separator{background-color:#626262;height:1px}"\
                               "QMenu::item:selected{background-color: #484848;}");
    tool_button->setMenu(button_menu);

    tool_button = new QToolButton;
    tool_button->setFixedSize(50,45);
    tool_button->setPopupMode(QToolButton::InstantPopup);
    tool_button->setStyleSheet("QToolButton::menu-indicator{image:none;}"\
                               "QToolButton{background:url(':/icons/205.png');border:0px;}"\
                               "QToolButton::pressed{background:url(':/icons/105.png')}"\
                               "QToolButton::hover{background:url(':/icons/305.png')}"\
                               "QMenu{background:#292929;color:white}");
    categories_toolbar->addWidget(tool_button);
    button_menu = new QMenu();
    button_menu->addActions(polyline_actions1);
    button_menu->addSeparator();
    button_menu->addActions(polyline_actions2);
    button_menu->addSeparator();
    button_menu->addActions(polyline_actions3);
    button_menu->setStyle(pStyle);
    button_menu->setStyleSheet("QMenu{background:#292929;color:white}"\
                       "QMenu::separator{background-color:#626262;height:1px}"\
                               "QMenu::item:selected{background-color: #484848;}");
    tool_button->setMenu(button_menu);

    tool_button = new QToolButton;
    tool_button->setFixedSize(50,45);
    tool_button->setPopupMode(QToolButton::InstantPopup);
    tool_button->setStyleSheet("QToolButton::menu-indicator{image:none;}"\
                               "QToolButton{background:url(':/icons/206.png');border:0px;}"\
                               "QToolButton::pressed{background:url(':/icons/106.png')}"\
                               "QToolButton::hover{background:url(':/icons/306.png')}"\
                               "QMenu{background:#292929;color:white}");
    categories_toolbar->addWidget(tool_button);
    button_menu = new QMenu();
    button_menu->addActions(select_actions1);
    button_menu->addSeparator();
    button_menu->addActions(select_actions2);
    button_menu->addSeparator();
    button_menu->addActions(select_actions3);
    button_menu->addSeparator();
    button_menu->addActions(select_actions4);
    button_menu->setStyle(pStyle);
    button_menu->setStyleSheet("QMenu{background:#292929;color:white}"\
                       "QMenu::separator{background-color:#626262;height:1px}"\
                               "QMenu::item:selected{background-color: #484848;}");
    tool_button->setMenu(button_menu);
//    tool_button->addActions(select_actions1);
//    tool_button->addActions(select_actions2);
//    tool_button->addActions(select_actions3);
//    tool_button->addActions(select_actions4);

    tool_button = new QToolButton;
    tool_button->setFixedSize(50,45);
    tool_button->setPopupMode(QToolButton::InstantPopup);
    tool_button->setStyleSheet("QToolButton::menu-indicator{image:none;}"\
                               "QToolButton{background:url(':/icons/207.png');border:0px;}"\
                               "QToolButton::pressed{background:url(':/icons/107.png')}"\
                               "QToolButton::hover{background:url(':/icons/307.png')}"\
                               "QMenu{background:#292929;color:white}");
    categories_toolbar->addWidget(tool_button);
    button_menu = new QMenu();
    button_menu->addActions(dimension_actions);
    button_menu->setStyle(pStyle);
    button_menu->setStyleSheet("QMenu{background:#292929;color:white}"\
                       "QMenu::separator{background-color:#626262;height:1px}"\
                               "QMenu::item:selected{background-color: #484848;}");
    tool_button->setMenu(button_menu);



    tool_button = new QToolButton;//修改
    tool_button->setFixedSize(50,45);
    tool_button->setPopupMode(QToolButton::InstantPopup);
    tool_button->setStyleSheet("QToolButton::menu-indicator{image:none;}"\
                               "QToolButton{background:url(':/icons/208.png');border:none;}"\
                               "QToolButton::pressed{background:url(':/icons/109.png')}"\
                               "QToolButton::hover{background:url(':/icons/308.png')}"\
                               "QMenu{background:#292929;color:white}");
    categories_toolbar->addWidget(tool_button);

    button_menu = new QMenu();
    button_menu->setStyle(pStyle);
    button_menu->setStyleSheet("QMenu{background:#292929;color:white}"\
                       "QMenu::separator{background-color:#626262;height:1px}"\
                               "QMenu::item:selected{background-color: #484848;}");

//    tool_button->addActions(modify_actions);
      button_menu->addAction(a_map["ModifyMove"]);
      button_menu->addAction(a_map["ModifyMoveRotate"]);
      button_menu->addAction(a_map["ModifyRotate"]);
      button_menu->addAction(a_map["ModifyRotate2"]);
      button_menu->addAction(a_map["ModifyRevertDirection"]);
//      button_menu->addAction(a_map["ModifyScale"]);
      button_menu->addAction(a_map["ModifyMirror"]);
      button_menu->addSeparator();



      button_menu->addAction(a_map["ModifyTrim"]);
      button_menu->addAction(a_map["ModifyTrim2"]);
      button_menu->addAction(a_map["ModifyTrimAmount"]);
      button_menu->addAction(a_map["ModifyOffset"]);
      button_menu->addAction(a_map["ModifyStretch"]);

      button_menu->addSeparator();


      button_menu->addAction(a_map["ModifyRound"]);
      button_menu->addAction(a_map["ModifyBevel"]);
      button_menu->addSeparator();

      button_menu->addAction(a_map["ModifyEntity"]);
      button_menu->addAction(a_map["ModifyAttributes"]);
      button_menu->addAction(a_map["ModifyExplodeText"]);
      button_menu->addSeparator();

      button_menu->addAction(a_map["ModifyDeleteQuick"]);
      tool_button->setMenu(button_menu);


    tool_button = new QToolButton;
    tool_button->setFixedSize(50,45);
    tool_button->setPopupMode(QToolButton::InstantPopup);
    tool_button->setStyleSheet("QToolButton::menu-indicator{image:none;}"\
                               "QToolButton{background:url(':/icons/209.png');border:0}"\
                               "QToolButton::pressed{background:url(':/icons/108.png')}"\
                               "QToolButton::hover{background:url(':/icons/309.png');border:none}"\
                               "QMenu{background:#292929;color:white}"\
                               "QMenu::item:selected{background-color: #484848;}");
    categories_toolbar->addWidget(tool_button);
    tool_button->addActions(info_actions);
    categories_toolbar->setMovable(false);

    categories_toolbar->setStyleSheet("QToolBar#categories_toolbar{background-color:#4d4d4d;margin-top:0px;border-top: 0.5px solid #4d4d4d;border-bottom:1px solid #707070;}");

    main_window->addToolBar(Qt::LeftToolBarArea, categories_toolbar);
//    main_window->getAppWindow()->setStyleSheet("background-color:red");
    return categories_toolbar;
}




void LC_WidgetFactory::createMenus(QMenuBar* menu_bar)
{
//    menu_bar->setFixedSize(0,0);
    // <[~ File ~]>
    menu_bar->setFixedSize(1000,29);
    file_menu = new QMenu(QC_ApplicationWindow::tr("&File"), menu_bar);

//    file_menu->setObjectName("File");
//    file_menu->setTearOffEnabled(true);
//    file_menu->addAction(a_map["FileNew"]);
//    file_menu->addAction(a_map["FileNewTemplate"]);
//    file_menu->addAction(a_map["FileOpen"]);
//    file_menu->addSeparator();
//    file_menu->addAction(a_map["FileSave"]);
//    file_menu->addAction(a_map["FileSaveAs"]);
//    file_menu->addAction(a_map["FileSaveAll"]);
//    file_menu->addSeparator();
//    sub_menu = file_menu->addMenu(QIcon(":/icons/import.svg"), QC_ApplicationWindow::tr("Import"));
//    sub_menu->setObjectName("Import");
//    sub_menu->addAction(a_map["DrawImage"]);
//    sub_menu->addAction(a_map["BlocksImport"]);
//    sub_menu = file_menu->addMenu(QIcon(":/icons/export.svg"), QC_ApplicationWindow::tr("Export"));
//    sub_menu->setObjectName("Export");
//    sub_menu->addAction(a_map["FileExportMakerCam"]);
//    sub_menu->addAction(a_map["FilePrintPDF"]);
//    sub_menu->addAction(a_map["FileExport"]);
//    file_menu->addSeparator();
//    file_menu->addAction(a_map["FilePrint"]);
//    file_menu->addAction(a_map["FilePrintPreview"]);
//    file_menu->addSeparator();
//    file_menu->addAction(a_map["FileClose"]);
//    file_menu->addAction(a_map["FileCloseAll"]);
//    file_menu->addAction(a_map["FileQuit"]);
//    file_menu->addSeparator();

    // <[~ 一级菜单绘图 ~]>

//    QMenu* settings_menu = new QMenu(QC_ApplicationWindow::tr("&Mapping"), menu_bar);
//    settings_menu->setObjectName("options_menu");
//    settings_menu->setTearOffEnabled(true);
    // <[~ 线 ~]>
//    sub_menu = settings_menu->addMenu(QC_ApplicationWindow::tr("&Line"));
//    sub_menu->setIcon(QIcon(":/icons/line.svg"));
//    sub_menu->setObjectName("Line");
//    sub_menu->addActions(line_actions);
    // <[~ Circles ~]>
//    sub_menu = settings_menu->addMenu(QC_ApplicationWindow::tr("&Circle"));
//    sub_menu->setIcon(QIcon(":/icons/circle.svg"));
//    sub_menu->setObjectName("Circle");
//    sub_menu->addActions(circle_actions);
    // <[~ Curves ~]>
//    sub_menu = settings_menu->addMenu(QC_ApplicationWindow::tr("&Curve"));
//    sub_menu->setIcon(QIcon(":/icons/line_freehand.svg"));
//    sub_menu->setObjectName("Curve");
//    sub_menu->addActions(curve_actions);
    // <[~ Ellipses ~]>
//    sub_menu = settings_menu->addMenu(QC_ApplicationWindow::tr("&Ellipse"));
//    sub_menu->setIcon(QIcon(":/icons/ellipses.svg"));
//    sub_menu->setObjectName("Ellipse");
//    sub_menu->addActions(ellipse_actions);
    // <[~ Polylines ~]>
//    sub_menu = settings_menu->addMenu(QC_ApplicationWindow::tr("&Polyline"));
//    sub_menu->setIcon(QIcon(":/icons/polylines_polyline.svg"));
//    sub_menu->setObjectName("Polyline");
//    sub_menu->addActions(polyline_actions);
    // <[~ 标注 ~]>
//    QMenu* dimension_menu = settings_menu->addMenu(QC_ApplicationWindow::tr("Dime&nsion"));
//    dimension_menu->setIcon(QIcon(":/icons/dim_horizontal.svg"));
//    dimension_menu->setObjectName("dimension_menu");
//    dimension_menu->setTearOffEnabled(true);
//    dimension_menu->addActions(dimension_actions);
    // <[~ 修改 ~]>
//    QMenu* modify_menu = settings_menu->addMenu(QC_ApplicationWindow::tr("&Modify"));
//    modify_menu->setIcon(QIcon(":/icons/move_rotate.svg"));
//    modify_menu->setObjectName("Modify");
//    modify_menu->setTearOffEnabled(true);
////    modify_menu->addMenu(order_menu);
//    modify_menu->addActions(modify_actions);
    // <[~ 对齐 ~]>
//    QMenu* align_menu = settings_menu->addMenu(QC_ApplicationWindow::tr("&Align"));
//    align_menu->setIcon(QIcon(":/icons/对齐.svg"));
//    align_menu->setObjectName("Align");
//    align_menu->setTearOffEnabled(true);
//    align_menu->addActions(align_actions);
    //align_menu->addAction(a_map["AlignRight"]);
    // <[~ 查询 ~]>
//    QMenu* info_menu = settings_menu->addMenu(QC_ApplicationWindow::tr("&Info"));
//    info_menu->setIcon(QIcon(":/icons/measure.svg"));
//    info_menu->setObjectName("Info");
//    info_menu->setTearOffEnabled(true);
//    info_menu->addActions(info_actions);


    //点
//    settings_menu->addAction(a_map["DrawPoint"]);
//    //尺寸
//    settings_menu->setObjectName("Size");
//    settings_menu->setTearOffEnabled(true);
//    settings_menu->addAction(a_map["Size"]);
//    //文本段和填充
//    settings_menu->addAction(a_map["DrawMText"]);
//    settings_menu->addAction(a_map["DrawText"]);
//    settings_menu->addAction(a_map["DrawHatch"]);



    // <[~ 一级菜单工艺 ~]>

//    QMenu* technology_menu = new QMenu(QC_ApplicationWindow::tr("&Technology"), menu_bar);
//    technology_menu->setObjectName("Edit");
//    technology_menu->setTearOffEnabled(true);
    // 二级菜单引线
/*    QMenu* Leadwire_menu = technology_menu->addMenu(QC_ApplicationWindow::tr("Lead &Wire"));
    Leadwire_menu->setIcon(QIcon(":/icons/引线.svg"));
    Leadwire_menu->setObjectName("Leadwire");
    Leadwire_menu->setTearOffEnabled(true);
    Leadwire_menu->addAction(a_map["SetLeadWire"]);  //设置引入引出线
    Leadwire_menu->addAction(a_map["DistinguishBetweenInternalAndExternalModels"]);*/  //区分内外模
    //二级菜单清除
//    QMenu* clean_menu = technology_menu->addMenu(QC_ApplicationWindow::tr("&Clean"));
//    clean_menu->setIcon(QIcon(":/icons/清除.svg"));
//    clean_menu->setObjectName("Clean");
//    clean_menu->setTearOffEnabled(true);
//    clean_menu->addAction(a_map["CleanLeadWire"]);  //清除引入引出线
//    clean_menu->addAction(a_map["CleanWeilian"]);  //清除微连
//    clean_menu->addAction(a_map["CancellationOfCompensation"]);  //取消补偿
    //工艺设置
//    QMenu* ProcessSetup_menu = technology_menu->addMenu(QC_ApplicationWindow::tr("&ProcessSetup"));
//    ProcessSetup_menu->setIcon(QIcon(":/icons/工艺设置.svg"));
//    ProcessSetup_menu->setObjectName("ProcessSetup");
//    ProcessSetup_menu->setTearOffEnabled(true);
//    ProcessSetup_menu->addAction(a_map["StartingPoint"]);  //起点
//    ProcessSetup_menu->addAction(a_map["Compensate"]);  //补偿
//    ProcessSetup_menu->addAction(a_map["VaginalIncision"]);  //阴切
//    ProcessSetup_menu->addAction(a_map["YangCut"]);  //阳切
//    ProcessSetup_menu->addAction(a_map["Weilian"]);  //微连
//    ProcessSetup_menu->addAction(a_map["AutoWeilian"]);  //自动微连
//    ProcessSetup_menu->addAction(a_map["ExplodeWeilian"]);  //炸开微连
//    ProcessSetup_menu->addAction(a_map["CoolingPoint"]);  //冷却点
//    ProcessSetup_menu->addAction(a_map["CleanCoolingPoint"]);  //清除冷却点
    //工具
//    QMenu* tool_menu = technology_menu->addMenu(QC_ApplicationWindow::tr("&Tool"));
//    tool_menu->setIcon(QIcon(":/icons/工具.svg"));
//    tool_menu->setObjectName("Tool");
//    tool_menu->setTearOffEnabled(true);

//    QMenu *measure_menu = tool_menu->addMenu(QC_ApplicationWindow::tr("&Measure"));   //添加三级菜单栏
//    measure_menu->setIcon(QIcon(":/icons/测量.svg"));
//    measure_menu->setObjectName("Measure");
//    measure_menu->setTearOffEnabled(true);
//    measure_menu->addAction(a_map["InfoDist"]);  //点到点
//    measure_menu->addAction(a_map["InfoTotalLength"]);  //选中实体的

//    QMenu *flyingcutting_menu = tool_menu->addMenu(QC_ApplicationWindow::tr("Flying &Cutting"));   //添加三级菜单栏
//    flyingcutting_menu->setIcon(QIcon(":/icons/飞切.svg"));
//    flyingcutting_menu->setObjectName("FlyingCutting");
//    flyingcutting_menu->setTearOffEnabled(true);
//    flyingcutting_menu->addAction(a_map["LinearFlightCutting"]);  //直线飞行切割
//    flyingcutting_menu->addAction(a_map["ArcFlyingCutting"]);  //圆弧飞行切割



    // <[~ 一级菜单数控 ~]>

//    QMenu* plugins_menu = new QMenu(QC_ApplicationWindow::tr("numerical&Control"), menu_bar);
//    plugins_menu->setObjectName("plugins_menu");
//    plugins_menu->setTearOffEnabled(true);
//    plugins_menu->addAction(a_map["OutputNC"]);

    // <[~ 一级菜单视图 ~]>

//    QMenu* view_menu = new QMenu(QC_ApplicationWindow::tr("&View"), menu_bar);
//    view_menu->setObjectName("view_menu");
//    view_menu->setTearOffEnabled(true);
//    view_menu->addAction(a_map["Fullscreen"]);
//    view_menu->addAction(a_map["ViewStatusBar"]);
//    view_menu->addAction(a_map["ViewGrid"]);
//    view_menu->addAction(a_map["ViewDraft"]);
//    view_menu->addSeparator();
//    view_menu->addAction(a_map["ZoomRedraw"]);
//    view_menu->addAction(a_map["ZoomIn"]);
//    view_menu->addAction(a_map["ZoomOut"]);
//    view_menu->addAction(a_map["ZoomAuto"]);
//    view_menu->addAction(a_map["ZoomPrevious"]);
//    view_menu->addAction(a_map["ZoomWindow"]);
//    view_menu->addAction(a_map["ZoomPan"]);

    // <[~ 一级菜单排样 ~]>
//    QMenu* layout_menu = new QMenu(QC_ApplicationWindow::tr("&Layout"), menu_bar);
//    layout_menu->setObjectName("layout_menu");
//    layout_menu->setTearOffEnabled(true);

    // 二级菜单零件
//    QMenu* spareParts_menu = layout_menu->addMenu(QC_ApplicationWindow::tr("Spare &Parts"));
//    spareParts_menu->setIcon(QIcon(":/icons/零件.svg"));
//    spareParts_menu->setObjectName("spareParts");
//    spareParts_menu->setTearOffEnabled(true);
//    spareParts_menu->addActions(spareParts_actions);

    //二级菜单板材
//    QMenu* board_menu = layout_menu->addMenu(QC_ApplicationWindow::tr("&Board"));
//    board_menu->setIcon(QIcon(":/icons/板材.svg"));
//    board_menu->setObjectName("board");
//    board_menu->setTearOffEnabled(true);
//    board_menu->addActions(board_actions);

    //二级菜单排样
//    QMenu* sunlayout_menu = layout_menu->addMenu(QC_ApplicationWindow::tr("&Layout"));
//    sunlayout_menu->setIcon(QIcon(":/icons/排样.svg"));
//    sunlayout_menu->setObjectName("sunLayout");
//    sunlayout_menu->setTearOffEnabled(true);
//    sunlayout_menu->addActions(layout_actions);

    // <[~ 二级菜单选择 ~]>
/*
    QMenu* select_menu = layout_menu->addMenu(QC_ApplicationWindow::tr("&Select"));
    select_menu->setIcon(QIcon(":/icons/select.svg"));
    select_menu->setObjectName("Select");
    select_menu->setTearOffEnabled(true);
    select_menu->addActions(select_actions);
    select_menu->addAction(a_map["EditKillAllActions"]);
    select_menu->addSeparator();
    select_menu->addAction(a_map["EditUndo"]);
    select_menu->addAction(a_map["EditRedo"]);
    select_menu->addSeparator();
    select_menu->addAction(a_map["EditCut"]);
    select_menu->addAction(a_map["EditCopy"]);
    select_menu->addAction(a_map["EditPaste"]);
    select_menu->addAction(a_map["ModifyDeleteQuick"]);
*/



    //二级菜单群组
//    QMenu* group_menu = layout_menu->addMenu(QC_ApplicationWindow::tr("&Group"));
//    group_menu->setIcon(QIcon(":/icons/群组.svg"));
//    group_menu->setObjectName("Group");
//    group_menu->setTearOffEnabled(true);
//    group_menu->addActions(group_actions);

    //二级菜单阵列
//    QMenu* array_menu = layout_menu->addMenu(QC_ApplicationWindow::tr("&Array"));
//    array_menu->setIcon(QIcon(":/icons/阵列.svg"));
//    array_menu->setObjectName("Array");
//    array_menu->setTearOffEnabled(true);
//    array_menu->addActions(array_actions);

    //二级菜单优化
//    QMenu* optimization_menu = layout_menu->addMenu(QC_ApplicationWindow::tr("&Optimization"));
//    optimization_menu->setIcon(QIcon(":/icons/优化.svg"));
//    optimization_menu->setObjectName("Optimization");
//    optimization_menu->setTearOffEnabled(true);
//    optimization_menu->addActions(optimization_actions);

    //二级菜单排序
//    QMenu* sort_menu = layout_menu->addMenu(QC_ApplicationWindow::tr("&Sort"));
//    sort_menu->setIcon(QIcon(":/icons/排序.svg"));
//    sort_menu->setObjectName("Sort");
//    sort_menu->setTearOffEnabled(true);
//    sort_menu->addActions(sort_actions);
//    sort_menu->addSeparator();
//    sort_menu->addAction(a_map["PreventSortingFromChangingDirection"]);   //禁止排序改变方向
//    sort_menu->addAction(a_map["InnerOuterModelOfSortingTimeZone"]);   //排序时区分内外模
//    sort_menu->addAction(a_map["TheOutermostLayerIsIncised"]); //最外层为阴切

    //二级菜单共边
  //  layout_menu->addAction(a_map["CommonEdge"]);

    //二级菜单桥接
//    layout_menu->addAction(a_map["Bridging"]);


    // <[~ Order ~]>

//    QMenu* order_menu = new QMenu(QC_ApplicationWindow::tr("&Order"), menu_bar);
//    order_menu->setObjectName("order_menu");
//    order_menu->setTearOffEnabled(true);
//    order_menu->addAction(a_map["OrderTop"]);
//    order_menu->addAction(a_map["OrderBottom"]);
//    order_menu->addAction(a_map["OrderRaise"]);
//    order_menu->addAction(a_map["OrderLower"]);



    // <[~ Layer ~]>

//    QMenu* layer_menu = new QMenu(QC_ApplicationWindow::tr("&Layer"), menu_bar);
//    layer_menu->setObjectName("layer_menu");
//    layer_menu->setTearOffEnabled(true);
//    layer_menu->addActions(layer_actions);

    // <[~ Block ~]>

//    QMenu* block_menu = new QMenu(QC_ApplicationWindow::tr("&Block"), menu_bar);
//    block_menu->setObjectName("block_menu");
//    block_menu->setTearOffEnabled(true);
//    block_menu->addActions(block_actions);

    // <[~ Snapping ~]>

//    QMenu* snap_menu = new QMenu(QC_ApplicationWindow::tr("Sna&p"), menu_bar);
//    snap_menu->setObjectName("snap_menu");
//    snap_menu->setTearOffEnabled(true);
//    // QToolBar* snap_tb = main_window->findChild<QToolBar*>("snap_toolbar");
//    snap_menu->addActions(snap_toolbar->actions());

    // <[~ Drawings ~]>

//    windows_menu = new QMenu(QC_ApplicationWindow::tr("&Drawings"), menu_bar);
//    windows_menu->setObjectName("drawings_menu");
//    windows_menu->setTearOffEnabled(true);
//    windows_menu->addAction(a_map["Fullscreen"]); // temp way to show this menu on OS X

//    connect(windows_menu, SIGNAL(aboutToShow()),
//            main_window, SLOT(slotWindowsMenuAboutToShow()));

    // <[~ 一级菜单帮助 ~]>

    QMenu* help_menu = new QMenu(QC_ApplicationWindow::tr("&Help"), menu_bar);
    help_menu->setObjectName("Help");
    help_menu->setTearOffEnabled(true);

    help_menu->addAction(a_map["OptionsGeneral"]);
//    help_menu->addAction(a_map["OptionsDrawing"]);
//    help_menu->addAction(a_map["WidgetOptions"]);
//    help_menu->addAction(a_map["DeviceOptions"]);
//    help_menu->addAction(a_map["ReloadStyleSheet"]);

    QAction* wiki_link = new QAction(QC_ApplicationWindow::tr("Online"), main_window);
    connect(wiki_link, SIGNAL(triggered()), main_window, SLOT(invokeLinkList()));
    help_menu->addAction(wiki_link);

    help_menu->addSeparator();

    QAction* help_about = new QAction(QIcon(":/main/librecad.png"), QC_ApplicationWindow::tr("About"), main_window);
//    QAction* help_about = new QAction( QC_ApplicationWindow::tr("About"), main_window);
    connect(help_about, SIGNAL(triggered()), main_window, SLOT(showAboutWindow()));
    help_menu->addAction(help_about);

    QAction* license = new QAction(QObject::tr("License"), main_window);
    connect(license, SIGNAL(triggered()), main_window, SLOT(invokeLicenseWindow()));
    help_menu->addAction(license);



    // <[~ Test ~]>
//    QMenu* test_menu = new QMenu(QC_ApplicationWindow::tr("测试菜单"), menu_bar);
//    test_menu->setObjectName("test_menu");
//    test_menu->setTearOffEnabled(true);

//    QAction* show_test = new QAction(QC_ApplicationWindow::tr("ShowTestWidget"), main_window);
//    connect(show_test, SIGNAL(triggered()), main_window, SLOT(showtest()));
//    test_menu->addAction(show_test);


    // <[~ Widgets Menu ~]>

//    QMenu* widgets_menu = new QMenu("Widgets", menu_bar);
//    widgets_menu->setTearOffEnabled(true);

    // <[~ Dock Areas Menu ~]>

//    QMenu* dockareas_menu = new QMenu("Dock Areas", widgets_menu);

//    dockareas_menu->addAction(a_map["LeftDockAreaToggle"]);
//    dockareas_menu->addAction(a_map["RightDockAreaToggle"]);
//    dockareas_menu->addAction(a_map["TopDockAreaToggle"]);
//    dockareas_menu->addAction(a_map["BottomDockAreaToggle"]);
//    dockareas_menu->addAction(a_map["FloatingDockwidgetsToggle"]);

    // <[~ Dock Widgets Menu ~]>

//    QMenu* dockwidgets_menu = new QMenu(QC_ApplicationWindow::tr("Dock Wid&gets"), widgets_menu);
//    dockwidgets_menu->setObjectName("dockwidgets_menu");
//    dockwidgets_menu->setTearOffEnabled(true);

//    dockwidgets_menu->addSeparator();

//    QList<QDockWidget*> dockwidgets = main_window->findChildren<QDockWidget*>();

//    main_window->sortWidgetsByTitle(dockwidgets);

//    foreach (QDockWidget* dw, dockwidgets)
//    {
//        if (main_window->dockWidgetArea(dw) == Qt::RightDockWidgetArea)
//            dockwidgets_menu->addAction(dw->toggleViewAction());
//    }

//    dockwidgets_menu->addSeparator();

//    foreach (QDockWidget* dw, dockwidgets)
//    {
//        if (main_window->dockWidgetArea(dw) == Qt::LeftDockWidgetArea)
//            dockwidgets_menu->addAction(dw->toggleViewAction());
//    }

    // <[~ Toolbars Menu ~]>

//    QMenu* toolbars_menu = new QMenu(QC_ApplicationWindow::tr("&Toolbars"), widgets_menu);
//    toolbars_menu->setObjectName("toolbars_menu");
//    toolbars_menu->setTearOffEnabled(true);

//    QList<QToolBar*> toolbars = main_window->findChildren<QToolBar*>();

//    main_window->sortWidgetsByTitle(toolbars);

//    foreach (QToolBar* tb, toolbars)
//    {
//        toolbars_menu->addAction(tb->toggleViewAction());
//    }

//    widgets_menu->addMenu(dockareas_menu);
//    widgets_menu->addMenu(dockwidgets_menu);
//    widgets_menu->addMenu(toolbars_menu);
//    widgets_menu->addAction(a_map["InvokeMenuCreator"]);
//    widgets_menu->addAction(a_map["InvokeToolbarCreator"]);

    // <[~ MenuBar Layout~]>

//    menu_bar->addMenu(file_menu);
//    menu_bar->addMenu(view_menu);
/*    menu_bar->addMenu(settings_menu);
    menu_bar->addMenu(technology_menu);
    menu_bar->addMenu(view_menu);
    menu_bar->addMenu(plugins_menu);
//    menu_bar->addMenu(select_menu);
    menu_bar->addMenu(layout_menu);
//    menu_bar->addMenu(dimension_menu);
//    menu_bar->addMenu(modify_menu);
//    menu_bar->addMenu(snap_menu);
//    menu_bar->addMenu(info_menu);
//    menu_bar->addMenu(layer_menu);
//    menu_bar->addMenu(block_menu);
//    menu_bar->addMenu(widgets_menu);
//    menu_bar->addMenu(windows_menu);
    menu_bar->addMenu(help_menu);
//    menu_bar->addMenu(test_menu);*/
}

void LC_WidgetFactory::slotClose(bool isClose)
{
    if(isClose == true)
    {
       dock_part->close();
    }
}


void LC_WidgetFactory::slotFile(){
    widget = new QWidget(main_window);

/*
    QToolBar *tb1 = new QToolBar(widget);
    QLabel *Label1 = new QLabel(widget);
    Label1->setFixedSize(260,50);
    Label1->setText("文件");
    QFont font ( "SimSun", 17, 75);
    Label1->setStyleSheet("background:#4d4d4d;");
    Label1->setFont(font);
    Label1->setAlignment(Qt::AlignCenter);
    tb1->addWidget(Label1);
    tb1->setMovable(false);

//    QToolBar *tb2 = new QToolBar(widget);
    QToolButton *Label2 = new QToolButton;
    QFont font2 ( "SimSun", 11, 60);
    Label2->setFont(font2);
    Label2->setFixedSize(250,23);
    Label2->setDefaultAction(a_map["FileNew"]);
    Label2->setToolButtonStyle(Qt::ToolButtonTextOnly);
    Label2->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                          "QToolButton::hover{background:#4d4d4d;}"\
                          "QToolButton{border:0;}"\
                          "QToolTip{border: 0px solid;}");

    Label2->setToolTipDuration(1);

    tb1->addWidget(Label2);



    QToolButton *Label3 = new QToolButton;

    Label3->setFixedSize(250,23);
    Label3->setFont(font2);
   // Label3->addAction(a_map["FileNewTemplate"]);
    Label3->setDefaultAction(a_map["FileNewTemplate"]);
    Label3->setToolButtonStyle(Qt::ToolButtonTextOnly);
    Label3->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                          "QToolButton::hover{background:#4d4d4d;}"\
                          "QToolButton{border:0;}"\
                          "QToolTip{border: 0px solid;}");
    Label3->setToolTipDuration(1);
    tb1->addWidget(Label3);

    QToolButton *Label4 = new QToolButton;
    Label4->setText("打     开");
    Label4->setFixedSize(250,23);
    Label4->setFont(font2);
    Label4->setDefaultAction(a_map["FileOpen"]);
    Label4->setToolButtonStyle(Qt::ToolButtonTextOnly);
//    Label4->addAction(a_map["FileNewTemplate"]);
    Label4->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                          "QToolButton::hover{background:#4d4d4d;}"\
                          "QToolButton{border:0;}"\
                          "QToolTip{border: 0px solid;}");
    Label4->setToolTipDuration(1);
    tb1->addWidget(Label4);


    QToolButton *Label5 = new QToolButton;
  //  Label5->setText("保     存");
    Label5->setFixedSize(250,23);
  //  Label5->setStyleSheet("border-bottom:1px solid #4e4e4e;");
    Label5->setFont(font2);
    Label5->setDefaultAction(a_map["FileOpen"]);   //最近打开
    Label5->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                          "QToolButton::hover{background:#4d4d4d;}"\
                          "QToolButton{border:0;border-bottom:1px solid #4e4e4e;}"\
                          "QToolTip{border: 0px solid;}");
    Label5->setToolTipDuration(1);
    Label5->setToolButtonStyle(Qt::ToolButtonTextOnly);
    tb1->addWidget(Label5);

    QToolButton *Label6 = new QToolButton;
    Label6->setText("保     存");
    Label6->setFixedSize(250,23);
    Label6->setFont(font2);
    Label6->setDefaultAction(a_map["FileSave"]);
      Label6->setToolButtonStyle(Qt::ToolButtonTextOnly);
    Label6->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                          "QToolButton::hover{background:#4d4d4d;}"\
                          "QToolButton{border:0;}"\
                          "QToolTip{border: 0px solid;}");
    Label6->setToolTipDuration(1);

    tb1->addWidget(Label6);

    QToolButton *Label7 = new QToolButton;

    Label7->setFont(font2);
    Label7->setFixedSize(250,23);
    Label7->setDefaultAction(a_map["FileSaveAs"]);
    Label7->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                          "QToolButton::hover{background:#4d4d4d;}"\
                          "QToolButton{border:0;}"\
                          "QToolTip{border: 0px solid;}");
    Label7->setToolTipDuration(1);
    Label7->setToolButtonStyle(Qt::ToolButtonTextOnly);
    tb1->addWidget(Label7);

    QToolButton *Label8 = new QToolButton;
  //  Label8->setText("另 存 全 部");

    //Label8->setStyleSheet("border-bottom:1px solid #4e4e4e;");
    Label8->setFont(font2);
    Label8->setFixedSize(250,23);
    Label8->setDefaultAction(a_map["FileSaveAll"]);
    Label8->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                          "QToolButton::hover{background:#4d4d4d;}"\
                          "QToolButton{border:0;border-bottom:1px solid #4e4e4e;}"\
                          "QToolTip{border: 0px solid;}");
    Label8->setToolTipDuration(1);
    Label8->setToolButtonStyle(Qt::ToolButtonTextOnly);
    tb1->addWidget(Label8);

   // QToolBar
    QToolButton *Label9 = new QToolButton;
 //   Label9->setText("导     入");
    Label9->setFont(font2);
    Label9->setFixedSize(250,23);
    Label9->addAction(a_map["FileSaveAll"]);
    Label9->addAction(a_map["FileSaveAll"]);

//    Label9->setDefaultAction(a_map["FileSaveAll"]);
    Label9->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                          "QToolButton::hover{background:#4d4d4d;}"\
                          "QToolButton{border:0;}"\
                          "QToolTip{border: 0px solid;}");
//    Label9->setToolTipDuration(1);

//    Label9->setToolButtonStyle(Qt::ToolButtonTextOnly);

    tb1->addWidget(Label9);


    QToolButton *Label10 = new QToolButton;
    Label10->setText("导     出");

    Label10->setFont(font2);
    Label10->setFixedSize(250,23);
    Label10->setDefaultAction(a_map["FileSaveAll"]);
    Label10->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                          "QToolButton::hover{background:#4d4d4d;}"\
                          "QToolButton{border:0;border-bottom:1px solid #4e4e4e;}"\
                          "QToolTip{border: 0px solid;}");
    Label10->setToolTipDuration(1);
    Label10->setToolButtonStyle(Qt::ToolButtonTextOnly);
    tb1->addWidget(Label10);

    Label10->setStyleSheet("QLabel{border-bottom:1px solid #4e4e4e;}"\
                           "QLabel::hover{background:#4d4d4d;}");

    QLabel *Label11 = new QLabel;
    Label11->setText("打     印");
    Label11->setAlignment(Qt::AlignCenter);
    Label11->setFont(font2);
      Label11->setStyleSheet("QLabel::hover{background:#4d4d4d;}");
    tb1->addWidget(Label11);

    QLabel *Label12 = new QLabel;
    Label12->setText("打 印 预 览");
    Label12->setAlignment(Qt::AlignCenter);
    Label12->setFont(font2);
    tb1->addWidget(Label12);
    Label12->setStyleSheet("QLabel{border-bottom:1px solid #4e4e4e;}"\
                           "QLabel::hover{background:#4d4d4d;}");

    QLabel *Label13 = new QLabel;
    Label13->setText("关     闭");
    Label13->setAlignment(Qt::AlignCenter);
    Label13->setFont(font2);
    tb1->addWidget(Label13);
    Label13->setStyleSheet("QLabel{border-bottom:1px solid #4e4e4e;}"\
                           "QLabel::hover{background:#4d4d4d;}");

    QLabel *Label14 = new QLabel;
    Label14->setFixedSize(250,1);
//    Label14->setAlignment(Qt::AlignCenter);
//    Label14->setFont(font2);
    tb1->addWidget(Label14);
    Label14->setStyleSheet("border-bottom:1px solid #4e4e4e;");

    QLabel *Label15 = new QLabel;
    Label15->setFixedSize(250,20);
    tb1->addWidget(Label15);
    Label15->setStyleSheet("border-bottom:1px solid #4e4e4e;");

    QHBoxLayout *layout4 = new QHBoxLayout;
    layout4->setMargin(0);
    QPushButton *button = new QPushButton("×退出");
    button->setFixedSize(75,25);
    button->setStyleSheet("QPushButton{background:#4d4d4d;border:1px solid #676767;}"\
                         "QPushButton::hover{background:#303030;}" );
    connect(button,SIGNAL(clicked()),this,SLOT(slotQuit()));

    layout4->addWidget(button);
    layout4->setAlignment(Qt::AlignRight);

//    tb1->addLayout(layout4);


    QLabel *Label16 = new QLabel;
    Label16->setFixedSize(250,50);
    tb1->addWidget(Label16);
*/

//    layout1->addLayout(layout2);
//    layout1->addLayout(layout3);


//    widget->setLayout(layout1);









  //  tb1->addActions(file_actions);

 //   layout3->addWidget(Label2);


    QVBoxLayout *layout1 = new QVBoxLayout(widget);
    layout1->setMargin(0);


    QHBoxLayout *layout2 = new QHBoxLayout;



//    layout2.set
    QLabel *Label1 = new QLabel(widget);
    Label1->setFixedSize(260,50);
    Label1->setText("文件");
    QFont font ( "SimSun", 17, 75);
    Label1->setStyleSheet("background:#4d4d4d;");
    Label1->setFont(font);
    Label1->setAlignment(Qt::AlignCenter);

    layout2->addWidget(Label1);


    QVBoxLayout *layout3 = new QVBoxLayout;

    QToolButton *Label2 = new QToolButton;
    QFont font2 ( "SimSun", 11, 60);
    Label2->setFont(font2);
    Label2->setFixedSize(250,23);
   Label2->setDefaultAction(a_map["FileNew"]);
    Label2->setToolButtonStyle(Qt::ToolButtonTextOnly);
    Label2->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                          "QToolButton::hover{background:#4d4d4d;}"\
                          "QToolButton{border:0;}"\
                          "QToolTip{border: 0px solid;}");
//Label2->setToolTip("");
Label2->setToolTipDuration(1);
    layout3->addWidget(Label2);


    QToolButton *Label3 = new QToolButton;

    Label3->setFixedSize(250,23);
    Label3->setFont(font2);
   // Label3->addAction(a_map["FileNewTemplate"]);
    Label3->setDefaultAction(a_map["FileNewTemplate"]);
    Label3->setToolButtonStyle(Qt::ToolButtonTextOnly);
    Label3->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                          "QToolButton::hover{background:#4d4d4d;}"\
                          "QToolButton{border:0;}"\
                          "QToolTip{border: 0px solid;}");
    Label3->setToolTipDuration(1);
    layout3->addWidget(Label3);

    QToolButton *Label4 = new QToolButton;
    Label4->setText("打     开");
    Label4->setFixedSize(250,23);
    Label4->setFont(font2);
    Label4->setDefaultAction(a_map["FileOpenPreview"]);
    Label4->setToolButtonStyle(Qt::ToolButtonTextOnly);
//    Label4->addAction(a_map["FileNewTemplate"]);
    Label4->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                          "QToolButton::hover{background:#4d4d4d;}"\
                          "QToolButton{border:0;}"\
                          "QToolTip{border: 0px solid;}");
    Label4->setToolTipDuration(1);
    layout3->addWidget(Label4);


    QToolButton *Label5 = new QToolButton;
  //  Label5->setText("保     存");
    Label5->setFixedSize(250,23);
  //  Label5->setStyleSheet("border-bottom:1px solid #4e4e4e;");
    Label5->setFont(font2);
    Label5->setDefaultAction(a_map["FileOpen"]);   //最近打开
    Label5->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                          "QToolButton::hover{background:#4d4d4d;}"\
                          "QToolButton{border:0;border-bottom:1px solid #4e4e4e;}"\
                          "QToolTip{border: 0px solid;}");
    Label5->setToolTipDuration(1);
    Label5->setToolButtonStyle(Qt::ToolButtonTextOnly);
    layout3->addWidget(Label5);

    QToolButton *Label6 = new QToolButton;
    Label6->setText("保     存");
    Label6->setFixedSize(250,23);
    Label6->setFont(font2);
    Label6->setDefaultAction(a_map["FileSave"]);
      Label6->setToolButtonStyle(Qt::ToolButtonTextOnly);
    Label6->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                          "QToolButton::hover{background:#4d4d4d;}"\
                          "QToolButton{border:0;}"\
                          "QToolTip{border: 0px solid;}");
    Label6->setToolTipDuration(1);

    layout3->addWidget(Label6);

    QToolButton *Label7 = new QToolButton;

    Label7->setFont(font2);
    Label7->setFixedSize(250,23);
    Label7->setDefaultAction(a_map["FileSaveAs"]);
    Label7->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                          "QToolButton::hover{background:#4d4d4d;}"\
                          "QToolButton{border:0;}"\
                          "QToolTip{border: 0px solid;}");
    Label7->setToolTipDuration(1);
    Label7->setToolButtonStyle(Qt::ToolButtonTextOnly);
    layout3->addWidget(Label7);

    QToolButton *Label8 = new QToolButton;
  //  Label8->setText("另 存 全 部");

    //Label8->setStyleSheet("border-bottom:1px solid #4e4e4e;");
    Label8->setFont(font2);
    Label8->setFixedSize(250,23);
    Label8->setDefaultAction(a_map["FileSaveAll"]);
    Label8->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                          "QToolButton::hover{background:#4d4d4d;}"\
                          "QToolButton{border:0;border-bottom:1px solid #4e4e4e;}"\
                          "QToolTip{border: 0px solid;}");
    Label8->setToolTipDuration(1);
    Label8->setToolButtonStyle(Qt::ToolButtonTextOnly);
    layout3->addWidget(Label8);

   // QToolBar
    QToolButton *Label9 = new QToolButton;
    Label9->setText("导     入");
    Label9->setFont(font2);
    Label9->setFixedSize(250,23);
    Label9->setPopupMode(QToolButton::InstantPopup);
 //   Label9->addActions(file_actions);
    Label9->addAction(a_map["DrawImage"]);
    Label9->addAction(a_map["BlocksImport"]);
  //  Label9->setDefaultAction(a_map["FileSaveAs"]);


    Label9->setStyleSheet("QToolButton::menu-indicator{img:none;}"  \
                          "QToolButton::hover{background:#4d4d4d;}"\
                          "QToolButton{border:0;}"\
                          "QToolTip{border: 0px solid;}");
    Label9->setToolTipDuration(1);

//    Label9->setToolButtonStyle(Qt::ToolButtonTextOnly);

    layout3->addWidget(Label9);


    QToolButton *Label10 = new QToolButton;
    Label10->setText("导     出");

    Label10->setFont(font2);
    Label10->setFixedSize(250,23);
//    Label10->setDefaultAction(a_map["FileSaveAll"]);
    Label10->setPopupMode(QToolButton::InstantPopup);
    Label10->addAction(a_map["FileExportMakerCam"]);
    Label10->addAction(a_map["FilePrintPDF"]);
    Label10->addAction(a_map["FileExport"]);

    Label10->setStyleSheet("QToolButton::menu-indicator{image:none}"  \
                          "QToolButton::hover{background:#4d4d4d;}"\
                          "QToolButton{border:0;border-bottom:1px solid #4e4e4e;}"\
                          "QToolTip{border: 0px solid;}");
    Label10->setToolTipDuration(1);
    Label10->setToolButtonStyle(Qt::ToolButtonTextOnly);
    layout3->addWidget(Label10);

//    Label10->setStyleSheet("QLabel{border-bottom:1px solid #4e4e4e;}"\
//                           "QLabel::hover{background:#4d4d4d;}");

    QLabel *Label11 = new QLabel;
    Label11->setText("打     印");
    Label11->setAlignment(Qt::AlignCenter);
    Label11->setFont(font2);
      Label11->setStyleSheet("QLabel::hover{background:#4d4d4d;}");
    layout3->addWidget(Label11);

    QLabel *Label12 = new QLabel;
    Label12->setText("打 印 预 览");
    Label12->setAlignment(Qt::AlignCenter);
    Label12->setFont(font2);
    layout3->addWidget(Label12);
    Label12->setStyleSheet("QLabel{border-bottom:1px solid #4e4e4e;}"\
                           "QLabel::hover{background:#4d4d4d;}");

    QLabel *Label13 = new QLabel;
    Label13->setText("关     闭");
    Label13->setAlignment(Qt::AlignCenter);
    Label13->setFont(font2);
    layout3->addWidget(Label13);
    Label13->setStyleSheet("QLabel{border-bottom:1px solid #4e4e4e;}"\
                           "QLabel::hover{background:#4d4d4d;}");

    QLabel *Label14 = new QLabel;
    Label14->setFixedSize(250,1);
//    Label14->setAlignment(Qt::AlignCenter);
//    Label14->setFont(font2);
    layout3->addWidget(Label14);
    Label14->setStyleSheet("border-bottom:1px solid #4e4e4e;");

    QLabel *Label15 = new QLabel;
    Label15->setFixedSize(250,20);
    layout3->addWidget(Label15);
    Label15->setStyleSheet("border-bottom:1px solid #4e4e4e;");

    QHBoxLayout *layout4 = new QHBoxLayout;
    layout4->setMargin(0);
    QPushButton *button = new QPushButton("×退出");
    button->setFixedSize(75,25);
    button->setStyleSheet("QPushButton{background:#4d4d4d;border:1px solid #676767;}"\
                         "QPushButton::hover{background:#303030;}" );
    connect(button,SIGNAL(clicked()),this,SLOT(slotQuit()));

    layout4->addWidget(button);
    layout4->setAlignment(Qt::AlignRight);

    layout3->addLayout(layout4);


    QLabel *Label16 = new QLabel;
    Label16->setFixedSize(250,50);
    layout3->addWidget(Label16);


    layout1->addLayout(layout2);
    layout1->addLayout(layout3);


    widget->setLayout(layout1);
//    widget->addToolBar(Qt::LeftToolBarArea,tb1);

    widget->setWindowFlags(Qt::FramelessWindowHint);
    widget->setGeometry(0,30,250,530);
    widget->setStyleSheet("background:#303030;color:white;");
    widget->show();
}
void LC_WidgetFactory::slotQuit(){
    widget->close();
}


void LC_WidgetFactory::slotParts(){

    if(tool_button2->actions().empty()){
        part_widget->setFixedSize(325,850);
        tool_button2->addAction(a_map["FileOpenPreview"]);
        tool_button2->addAction(a_map["AddFactory"]);
//        part_widget->addWidget(partWidget);
        tool_button2->click();

        tool_button2->removeAction(a_map["FileOpenPreview"]);
        tool_button2->removeAction(a_map["AddFactory"]);
    }

}





void LC_WidgetFactory::slotCloseParts(){
    part_widget->setFixedSize(0,0);
}


//bool LC_WidgetFactory::eventFilter(QObject *obj, QEvent *event)
//{
//    if((obj == Label9)&&(event->type() == QEvent::HoverMove)) {
//            menu11->show();
//            Label9->setFocus();
//    }else if((obj == Label9)&&(event->type() == QEvent::HoverLeave)){
//            menu11->hide();
//    }
//    qDebug()<<"xxx";
//    return QObject::eventFilter(obj,event);
//}
