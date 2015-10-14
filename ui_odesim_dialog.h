/********************************************************************************
** Form generated from reading UI file 'odesim_dialog.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_ODESIM_DIALOG_H
#define UI_ODESIM_DIALOG_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QHeaderView>
#include <QtGui/QMainWindow>
#include <QtGui/QPushButton>
#include <QtGui/QStatusBar>
#include <QtGui/QWidget>
#include <SimVisWidget.h>

QT_BEGIN_NAMESPACE

class Ui_ODEDialog
{
public:
    QWidget *mCentralWidget;
    CSimVisWidget *mGLWidget;
    QPushButton *mBtnStartPause;
    QCheckBox *mCbxEnableGraphics;
    QCheckBox *mCbxSlowSimulation;
    QStatusBar *mStatusBar;

    void setupUi(QMainWindow *ODEDialog)
    {
        if (ODEDialog->objectName().isEmpty())
            ODEDialog->setObjectName(QString::fromUtf8("ODEDialog"));
        ODEDialog->resize(800, 590);
        mCentralWidget = new QWidget(ODEDialog);
        mCentralWidget->setObjectName(QString::fromUtf8("mCentralWidget"));
        mGLWidget = new CSimVisWidget(mCentralWidget);
        mGLWidget->setObjectName(QString::fromUtf8("mGLWidget"));
        mGLWidget->setGeometry(QRect(10, 10, 781, 511));
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(mGLWidget->sizePolicy().hasHeightForWidth());
        mGLWidget->setSizePolicy(sizePolicy);
        mBtnStartPause = new QPushButton(mCentralWidget);
        mBtnStartPause->setObjectName(QString::fromUtf8("mBtnStartPause"));
        mBtnStartPause->setGeometry(QRect(10, 530, 92, 28));
        QSizePolicy sizePolicy1(QSizePolicy::Minimum, QSizePolicy::Maximum);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(mBtnStartPause->sizePolicy().hasHeightForWidth());
        mBtnStartPause->setSizePolicy(sizePolicy1);
        mCbxEnableGraphics = new QCheckBox(mCentralWidget);
        mCbxEnableGraphics->setObjectName(QString::fromUtf8("mCbxEnableGraphics"));
        mCbxEnableGraphics->setGeometry(QRect(240, 530, 131, 23));
        mCbxEnableGraphics->setChecked(true);
        mCbxSlowSimulation = new QCheckBox(mCentralWidget);
        mCbxSlowSimulation->setObjectName(QString::fromUtf8("mCbxSlowSimulation"));
        mCbxSlowSimulation->setGeometry(QRect(380, 530, 93, 23));
        ODEDialog->setCentralWidget(mCentralWidget);
        mStatusBar = new QStatusBar(ODEDialog);
        mStatusBar->setObjectName(QString::fromUtf8("mStatusBar"));
        ODEDialog->setStatusBar(mStatusBar);

        retranslateUi(ODEDialog);

        QMetaObject::connectSlotsByName(ODEDialog);
    } // setupUi

    void retranslateUi(QMainWindow *ODEDialog)
    {
        ODEDialog->setWindowTitle(QApplication::translate("ODEDialog", "MainWindow", 0, QApplication::UnicodeUTF8));
        mBtnStartPause->setText(QApplication::translate("ODEDialog", "Start", 0, QApplication::UnicodeUTF8));
        mCbxEnableGraphics->setText(QApplication::translate("ODEDialog", "Enable graphics", 0, QApplication::UnicodeUTF8));
        mCbxSlowSimulation->setText(QApplication::translate("ODEDialog", "Slow", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class ODEDialog: public Ui_ODEDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_ODESIM_DIALOG_H
