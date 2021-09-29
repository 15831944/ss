#ifndef LC_WIDGETFACTORY_H
#define LC_WIDGETFACTORY_H

#include <QMap>
#include <QObject>
#include <QString>
#include "lc_dockwidget.h"
#include "QPainter"
#include "title_bar.h"
#include "tab1.h"
#include "tab3.h"
#include "tab4.h"
#include "tab5.h"
#include "tab6.h"
#include "tab7.h"
#include "qstatusbar.h"
#include "qg_coordinatewidget.h"
#include "qg_dialogfactory.h"
#include "rs_dialogfactoryinterface.h"
#include"rs_dialogfactory.h"
#include "qg_graphicview.h"
#include "qc_applicationwindow.h"

//#include "sinsun/ss_partcontainer.h"

#include "sinsun/arrangement.h"
#include "sinsun/ss_arrangewidget.h"
#include "sinsun/ss_sortwidget.h"
#include "sinsun/ss_layercut.h"

#include "sinsun/xstest/ncfile.h"
//#include "sinsun/xstest/ptshowncfile.h"

class ss_PartWidget;


class QMenu;
class QAction;
class QMenuBar;
class QToolBar;
class QActionGroup;

class QG_PenToolBar;
class QG_SnapToolBar;
class QG_LayerWidget;
class QG_BlockWidget;
class QG_ActionHandler;
class QG_LibraryWidget;
class QG_CommandWidget;
class LC_CustomToolbar;
class QC_ApplicationWindow;
class LC_ActionGroupManager;
class QG_DialogFactory;

/**
 * creates the widgets and adds them to the main window;
 * it also "tags" a few widgets that the main window uses
 */
class LC_WidgetFactory : public QObject
{
    Q_OBJECT

public:

    LC_WidgetFactory(QC_ApplicationWindow* main_win,
                     QMap<QString, QAction*>& action_map,
                     LC_ActionGroupManager* agm);

    void createStandardToolbars(QG_ActionHandler* action_handler);
    void createCADToolbars();
    void createMenus(QMenuBar* menu_bar);
    void createLeftSidebar(int columns, int icon_size);
    void createRightSidebar(QG_ActionHandler* action_handler);
      void  addSidebar();
    QToolBar* createCategoriesToolbar();
//    bool eventFilter(QObject *obj, QEvent *event);
    // --- tagged widgets ---
 //   QStatusBar *statusBar() const;
    QG_SnapToolBar* snap_toolbar;
    QG_PenToolBar* pen_toolbar;
    QToolBar* options_toolbar;
    QMainWindow *fileWindow1;
    QG_LayerWidget* layer_widget;
    QG_BlockWidget* block_widget;
    QG_LibraryWidget* library_widget;
    QG_CommandWidget* command_widget;
    QToolBar* part_widget;
    QToolButton *tool_button2;
    ss_PartWidget * partWidget;


    QMenu* file_menu;
    QMenu* windows_menu;
    QG_CoordinateWidget *coordinateWidget;
    QWidget *widget;
    QWidget *widget2;
    QLabel *Label13;
    QMenu * menu11;
    QToolButton *Label5;
    QMainWindow *fileWindow;
    QToolButton *Label9;
private:
  //  QG_CoordinateWidget* coordinateWidget {nullptr};

    TitleBar *pTitleBar;
    void paintEvent(QPaintEvent *event);
    QC_ApplicationWindow* main_window;
    QMap<QString, QAction*>& a_map;
    LC_ActionGroupManager* ag_manager;
    LC_DockWidget* dock_part;

    QList<QAction*> file_actions;
    QList<QAction*> line_actions1;
    QList<QAction*> line_actions2;
    QList<QAction*> line_actions3;
    QList<QAction*> circle_actions1;
    QList<QAction*> circle_actions2;
    QList<QAction*> curve_actions1;
    QList<QAction*> curve_actions2;
    QList<QAction*> curve_actions3;
    QList<QAction*> curve_actions4;
    QList<QAction*> part_actions;
    QList<QAction*> ellipse_actions;
    QList<QAction*> polyline_actions1;
    QList<QAction*> polyline_actions2;
    QList<QAction*> polyline_actions3;
    QList<QAction*> spareParts_actions;
    QList<QAction*> select_actions1;
    QList<QAction*> select_actions2;
    QList<QAction*> select_actions3;
    QList<QAction*> select_actions4;
    QList<QAction*> select_actions;
    QList<QAction*> group_actions;
    QList<QAction*> array_actions;
    QList<QAction*> optimization_actions;
    QList<QAction*> sort_actions;
    QList<QAction*> board_actions;
    QList<QAction*> layout_actions;
    QList<QAction*> dimension_actions;
    QList<QAction*> modify_actions;
    QList<QAction*> align_actions;
    QList<QAction*> info_actions;
    QList<QAction*> layer_actions;
    QList<QAction*> block_actions;    

public slots:
    void slotClose(bool);
    void slotFile();
    void slotQuit();
    void slotParts();
    void slotCloseParts();

protected:

//    void windowclosed();
//    void windowmin();
//    void windowredu();
//      void mouseDoubleClickEvent(QMouseEvent *e);
//    virtual  void mouseMoveEvent(QMouseEvent * event) override;
};

#endif // LC_WIDGETFACTORY_H
