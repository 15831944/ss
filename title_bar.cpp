#include <QLabel>
#include <QPushButton>
#include <QHBoxLayout>
#include <QEvent>
#include <QMouseEvent>
#include <QApplication>
#include "title_bar.h"

#ifdef Q_OS_WIN
#pragma comment(lib, "user32.lib")
#include <qt_windows.h>
#include<QDebug>
#endif

TitleBar::TitleBar(QWidget *parent)
    : QWidget(parent)
{
    setFixedHeight(30);
    m_pTitleLabel = new QLabel(this);
    m_pMinimizeButton = new QPushButton(this);
    m_pMaximizeButton = new QPushButton(this);
    m_pCloseButton = new QPushButton(this);
    m_pMinimizeButton->setStyleSheet("QPushButton{background:url(':/main/min.png')no-repeat center center;background-color:transparent;border:0px;margin-top:1px;}"\
                                     "QPushButton:hover{background-color:rgb(229,229,229)}"\
                                     "QPushButton:pressed{background-color:rgb(202,202,203)}"
                                     );
    m_pMaximizeButton->setStyleSheet("QPushButton{background:url(':/main/max.png')no-repeat center center;background-color:transparent;border:0px;margin-top:1px;}"\
                                     "QPushButton:hover{background-color:rgb(229,229,229)}"\
                                     "QPushButton:pressed{background-color:rgb(202,202,203)}"
                                     );
    m_pCloseButton->setStyleSheet("QPushButton{background:url(':/main/close.png')no-repeat center center;background-color:transparent;border:0px;margin-top:1px;margin-right:1px;}"\
                                  "QPushButton:hover{background-color:rgb(209,48,49)}"\
                                  "QPushButton:pressed{background-color:rgb(209,48,49)}");
    this->resize(parent->width(),30);
    m_pTitleLabel->setStyleSheet("border:1px solid black;background-color:white");

    m_pTitleLabel->resize(this->width(),30);
    m_pTitleLabel->setAlignment(Qt::AlignHCenter|Qt::AlignVCenter);
    QPalette pal(palette());
    pal.setColor(QPalette::Background, QColor(255, 255, 255));
    setAutoFillBackground(true);
    setPalette(pal);


    m_pMinimizeButton->setFixedSize(50, 29);
    m_pMaximizeButton->setFixedSize(50, 29);
    m_pCloseButton->setFixedSize(50, 29);


    m_pTitleLabel->setObjectName("whiteLabel");
    m_pMinimizeButton->setObjectName("minimizeButton");
    m_pMaximizeButton->setObjectName("maximizeButton");
    m_pCloseButton->setObjectName("closeButton");

    m_pMinimizeButton->setToolTip("Minimize");
    m_pMaximizeButton->setToolTip("Maximize");
    m_pCloseButton->setToolTip("Close");

    QHBoxLayout *pLayout = new QHBoxLayout(this);
    pLayout->setContentsMargins(0,0,0,0);
//    pLayout->addWidget(m_pTitleLabel);
    pLayout->addStretch(1);
    pLayout->addWidget(m_pMinimizeButton);
    pLayout->setSpacing(0);
    pLayout->addWidget(m_pMaximizeButton);
    pLayout->setSpacing(0);
    pLayout->addWidget(m_pCloseButton);
    setLayout(pLayout);

//    connect(m_pMinimizeButton, SIGNAL(clicked(bool)), this, SLOT(onClicked()));
//    connect(m_pMaximizeButton, SIGNAL(clicked(bool)), this, SLOT(onClicked()));
//    connect(m_pCloseButton, SIGNAL(clicked(bool)), this, SLOT(onClicked()));

}

void TitleBar::setTitleName(QString filename)
{

}

TitleBar::~TitleBar()
{

}

//void TitleBar::mouseDoubleClickEvent(QMouseEvent *event)
//{
//    Q_UNUSED(event);
//    emit m_pMaximizeButton->clicked();
//}

//void TitleBar::mousePressEvent(QMouseEvent *event)
//{
//#ifdef Q_OS_WIN
//    if (ReleaseCapture())
//    {
//        QWidget *pWindow = this->window();
//        if (pWindow->isTopLevel())
//        {
//           SendMessage(HWND(pWindow->winId()), WM_SYSCOMMAND, SC_MOVE + HTCAPTION, 0);
//        }
//    }
//       event->ignore();
//#else
//#endif
//}

bool TitleBar::eventFilter(QObject *obj, QEvent *event)
{
    switch (event->type())
    {
    case QEvent::WindowTitleChange:
    {
        QWidget *pWidget = qobject_cast<QWidget *>(obj);
        if (pWidget)
        {
            m_pTitleLabel->setText(pWidget->windowTitle());
            return true;
        }
    }
    case QEvent::WindowIconChange:
    {
        QWidget *pWidget = qobject_cast<QWidget *>(obj);
        if (pWidget)
        {
            QIcon icon = pWidget->windowIcon();
            return true;
        }
    }
    case QEvent::WindowStateChange:
    case QEvent::Resize:
//        updateMaximize();
        return true;
    }
    return QWidget::eventFilter(obj, event);
}

//void TitleBar::onClicked()
//{
//    QPushButton *pButton = qobject_cast<QPushButton *>(sender());
//    QWidget *pWindow = this->window();
//    if (pWindow->isTopLevel())
//    {
//        if (pButton == m_pMinimizeButton)
//        {
//            pWindow->showMinimized();
//        }
//        else if (pButton == m_pMaximizeButton)
//        {
//            if(pWindow->isMaximized()){
//                pWindow->showNormal();
//                m_pMaximizeButton->setStyleSheet("QPushButton{background:url(':/main/max.png')no-repeat center center;background-color:transparent;border:0px;margin-top:1px}"\
//                                                 "QPushButton:pressed{background-color:rgb(202,202,203)}"\
//                                                 "QPushButton:hover{background-color:rgb(229,229,229)}");
//            }else{
//                pWindow->showMaximized();
//                m_pMaximizeButton->setStyleSheet("QPushButton{background:url(':/main/scale.png')no-repeat center center;background-color:transparent;border:0px;margin-top:1px}"\
//                                                 "QPushButton:pressed{background-color:rgb(202,202,203)}"\
//                                                 "QPushButton:hover{background-color:rgb(229,229,229)}");
//            }
//        }
//        else if (pButton == m_pCloseButton)
//        {
//            pWindow->close();
//        }
//    }
//}

//void TitleBar::updateMaximize()
//{
//    QWidget *pWindow = this->window();
//    if (pWindow->isTopLevel())
//    {
//        bool bMaximize = pWindow->isMaximized();
//        if (bMaximize)
//        {
//            m_pMaximizeButton->setToolTip(tr("Restore"));
//            m_pMaximizeButton->setProperty("maximizeProperty", "restore");
//        }
//        else
//        {
//            m_pMaximizeButton->setProperty("maximizeProperty", "maximize");
//            m_pMaximizeButton->setToolTip(tr("Maximize"));
//        }

//        m_pMaximizeButton->setStyle(QApplication::style());
//    }
//}

void TitleBar::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);
    m_pTitleLabel->resize(this->width(),30);
}


