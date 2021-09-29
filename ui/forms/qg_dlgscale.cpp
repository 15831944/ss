/****************************************************************************
**
** This file is part of the LibreCAD project, a 2D CAD program
**
** Copyright (C) 2010 R. van Twisk (librecad@rvt.dds.nl)
** Copyright (C) 2001-2003 RibbonSoft. All rights reserved.
**
**
** This file may be distributed and/or modified under the terms of the
** GNU General Public License version 2 as published by the Free Software 
** Foundation and appearing in the file gpl-2.0.txt included in the
** packaging of this file.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
** 
** You should have received a copy of the GNU General Public License
** along with this program; if not, write to the Free Software
** Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
**
** This copyright notice MUST APPEAR in all copies of the script!  
**
**********************************************************************/
#include "qg_dlgscale.h"

#include "rs_settings.h"
#include "rs_modification.h"
#include "rs_math.h"
#include<QDebug>
#include <QDoubleValidator>
#include"rs_vector.h"
#include "rs_block.h"
#include <QString>
#include <QAction>
#include "rs_graphic.h"
#include"rs_entity.h"

#include "intern/qc_actiongetselect.h"
/*
 *  Constructs a QG_DlgScale as a child of 'parent', with the
 *  name 'name' and widget flags set to 'f'.
 *
 *  The dialog will by default be modeless, unless you set 'modal' to
 *  true to construct a modal dialog.
 */

QG_DlgScale::QG_DlgScale(RS_EntityContainer *container,QWidget* parent, bool modal, Qt::WindowFlags fl)
    : QDialog(parent, fl)
{
    setModal(modal);
    setupUi(this);
    this->container = container;
    init();
}

/*
*  Destroys the object and frees any allocated resources
*/
QG_DlgScale::~QG_DlgScale()
{
    destroy();
    // no need to delete child widgets, Qt does it all for us
}

/*
*  Sets the strings of the subwidgets using the current
*  language.
*/
void QG_DlgScale::languageChange()
{
    retranslateUi(this);
}


void QG_DlgScale::on_leFactorX_textChanged(const QString& arg1)
{
    scaleFactorX=arg1;
    if(cbIsotropic->isChecked()) {
        scaleFactorY=QString::number((scaleFactorX.toDouble()/wholeWidth)*wholeHeigt);
        leFactorY->setText(scaleFactorY);
    }
}


void QG_DlgScale::on_leFactorY_textChanged(const QString& arg1)
{
    scaleFactorY=arg1;
}


void QG_DlgScale::on_cbIsotropic_toggled(bool checked)
{
    leFactorY->setDisabled(checked);
    leFactorY->setReadOnly(checked);
    if(checked) {
        //scaleFactorY=scaleFactorX;
        scaleFactorY=QString::number((scaleFactorX.toDouble()/wholeWidth)*wholeHeigt);
        leFactorY->setText(scaleFactorY);
        comboBox->hide();
        label->hide();
    }else{
        comboBox->show();
        label->show();
    }
}

void QG_DlgScale::init() {
    RS_BlockData db = RS_BlockData();
    RS_Block *b = new RS_Block(container,db);
    for(auto ec: container->getEntityList()){
        if (ec->isSelected()) {
            //selectedList.push_back(ec);
            if(container->countSelected()>1){
                b->addEntity(ec);
                //qDebug()<<container->countSelected();
            }
            else{
                scaleFactorX = QString::number(ec->getSize().x);
                scaleFactorY = QString::number(ec->getSize().y);
            }
            //qDebug()<<"坐标参考"<<ec->maxV.x<<ec->maxV.y<<ec->minV.x<<ec->minV.y;

        }
    }
    if(container->countSelected()>1){
       // qDebug()<<b->getSize().x<<" "<<b->getSize().y;
        scaleFactorX = QString::number(b->getSize().x);
        scaleFactorY = QString::number(b->getSize().y);
    }
    wholeHeigt = scaleFactorY.toDouble();
    wholeWidth = scaleFactorX.toDouble();

    RS_SETTINGS->beginGroup("/Modify");
    copies = RS_SETTINGS->readEntry("/ScaleCopies", "10");
    numberMode = RS_SETTINGS->readNumEntry("/ScaleMode", 0);
    isotropic =
            (bool)RS_SETTINGS->readNumEntry("/ScaleIsotropic", 1);
    /*  scaleFactorX=RS_SETTINGS->readEntry("/ScaleFactorX", "1.0");
    scaleFactorY=RS_SETTINGS->readEntry("/ScaleFactorY", "1.0");
   */ useCurrentLayer =
            (bool)RS_SETTINGS->readNumEntry("/ScaleUseCurrentLayer", 0);
    useCurrentAttributes =
            (bool)RS_SETTINGS->readNumEntry("/ScaleUseCurrentAttributes", 0);
    RS_SETTINGS->endGroup();

    switch (numberMode) {
    case 0:
        rbMove->setChecked(true);
        break;
    case 1:
        rbCopy->setChecked(true);
        break;
    case 2:
        rbMultiCopy->setChecked(true);
        break;
    default:
        break;
    }


    leNumber->setText(copies);
    cbIsotropic->setChecked(isotropic);
    /*
    leFactorX->setValidator(new QDoubleValidator(1.e-10,1.e+10,10,leFactorX));
    leFactorY->setValidator(new QDoubleValidator(1.e-10,1.e+10,10,leFactorY));*/
    leFactorX->setText(scaleFactorX);
    leFactorY->setDisabled(isotropic);
    leFactorY->setReadOnly(isotropic);
    if (isotropic) {
        /*  scaleFactorY=scaleFactorX;
        leFactorY->setText(scaleFactorY);*/
        comboBox->hide();
        label->hide();
    } //else {
    leFactorY->setText(scaleFactorY);
    // }
    cbCurrentAttributes->setChecked(useCurrentAttributes);
    cbCurrentLayer->setChecked(useCurrentLayer);
}

void QG_DlgScale::destroy() {
    RS_SETTINGS->beginGroup("/Modify");
    RS_SETTINGS->writeEntry("/ScaleCopies", leNumber->text());
    /*   RS_SETTINGS->writeEntry("/ScaleFactorX", leFactorX->text());
    RS_SETTINGS->writeEntry("/ScaleFactorY", leFactorY->text());
   */ RS_SETTINGS->writeEntry("/ScaleIsotropic",
                              (int)cbIsotropic->isChecked());
    if (rbMove->isChecked()) {
        numberMode = 0;
    } else if (rbCopy->isChecked()) {
        numberMode = 1;
    } else {
        numberMode = 2;
    }
    RS_SETTINGS->writeEntry("/ScaleMode", numberMode);
    RS_SETTINGS->writeEntry("/ScaleUseCurrentLayer",
                            (int)cbCurrentLayer->isChecked());
    RS_SETTINGS->writeEntry("/ScaleUseCurrentAttributes",
                            (int)cbCurrentAttributes->isChecked());
    RS_SETTINGS->endGroup();
    delete leFactorX->validator();
    delete leFactorY->validator();
}

void QG_DlgScale::setData(RS_ScaleData* d) {
    data = d;
}

void QG_DlgScale::updateData() {
    if (rbMove->isChecked()) {
        data->number = 0;
    } else if (rbCopy->isChecked()) {
        data->number = 1;
    } else {
        data->number = leNumber->text().toInt();
    }
    scaleFactorX=QString::number(leFactorX->text().toDouble()/wholeWidth);
    leFactorY->setDisabled(cbIsotropic->isChecked());
    if(cbIsotropic->isChecked()) {
        scaleFactorY=QString::number((leFactorX->text().toDouble()/wholeWidth));
        //leFactorY->setText(QString::number((leFactorX->text().toDouble()/wholeWidth)*wholeHeigt));
    } else {
        scaleFactorY=QString::number(leFactorY->text().toDouble()/wholeHeigt);
    }
    //qDebug()<<scaleFactorX<<scaleFactorY;
    bool okX;
    double sx=RS_Math::eval(scaleFactorX,&okX);
    bool okY;
    double sy=RS_Math::eval(scaleFactorY,&okY);
    if(okX && okY){
        data->factor = RS_Vector(sx,sy);
        data->useCurrentAttributes = cbCurrentAttributes->isChecked();
        data->useCurrentLayer = cbCurrentLayer->isChecked();
    }else{
        leFactorX->setText("1");
        leFactorY->setText("1");
        data->factor=RS_Vector(false);
    }
}

void QG_DlgScale::on_comboBox_currentIndexChanged(int index)
{
    qDebug()<<index;
    switch(index){
        case 1:leFactorX->setText("20");
            leFactorY->setText("20");break;
        case 2:leFactorX->setText("50");
            leFactorY->setText("50");break;
        case 3:leFactorX->setText("100");
            leFactorY->setText("100");break;
        case 4:leFactorX->setText("200");
            leFactorY->setText("200");break;
        case 5:leFactorX->setText(QString::number(0.5*wholeWidth));
            leFactorY->setText(QString::number(0.5*wholeHeigt));break;
        case 6:leFactorX->setText(QString::number(2*wholeWidth));
            leFactorY->setText(QString::number(2*wholeHeigt));break;
        case 7:leFactorX->setText(QString::number(4*wholeWidth));
            leFactorY->setText(QString::number(4*wholeHeigt));break;
        case 8:leFactorX->setText(QString::number(8*wholeWidth));
            leFactorY->setText(QString::number(8*wholeHeigt));break;
        case 9:leFactorX->setText(QString::number(10*wholeWidth));
            leFactorY->setText(QString::number(10*wholeHeigt));break;
        case 10:leFactorX->setText(QString::number(20*wholeWidth));
            leFactorY->setText(QString::number(20*wholeHeigt));break;
        default:break;
    }
}
