/****************************************************************************
**
** This file is part of the LibreCAD project, a 2D CAD program
**
** Copyright (C) 2010 R. van Twisk (librecad@rvt.dds.nl)
** Copyright (C) 2001-2003 RibbonSoft. All rights reserved.
** Copyright (C) 2015, 2016 ravas (github.com/r-a-v-a-s)
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
#include "qg_dlgoptionsgeneral.h"

#include <QMessageBox>
#include <QColorDialog>
#include "rs_system.h"
#include "rs_settings.h"
#include "rs_units.h"
#include "qg_filedialog.h"
#include "rs_debug.h"
#include<QTableWidget>
#include<QScrollBar>
#include<QDebug>

/*
 *  Constructs a QG_DlgOptionsGeneral as a child of 'parent', with the
 *  name 'name' and widget flags set to 'f'.
 *
 *  The dialog will by default be modeless, unless you set 'modal' to
 *  true to construct a modal dialog.
 */

int QG_DlgOptionsGeneral::current_tab = 0;

QG_DlgOptionsGeneral::QG_DlgOptionsGeneral(QWidget* parent, bool modal, Qt::WindowFlags fl)
    : QDialog(parent, fl)
{
    setModal(modal);
    setupUi(this);
    totalheight = 0;
    tabWidget->setCurrentIndex(current_tab);
    init();
    connect(variablefile_button, &QToolButton::clicked,
            this, &QG_DlgOptionsGeneral::setVariableFile);
    connect(fonts_button, &QToolButton::clicked,
            this, &QG_DlgOptionsGeneral::setFontsFolder);

    tabWidget->setStyleSheet("QTabBar::tab {width:0;background-color:transparen;color: white;font-size:16px;border:0}"
                                     "QTabBar::tab:selected {color:0ff ;font-size:25px}"\
                             "QTabWidget::pane {border: 1px solid #000000;background-color:#4E4E4E}"\
                             "QTabBar{background-color:#4E4E4E}");
    this->setStyleSheet("QG_DlgOptionsGeneral{background-color:#4E4E4E;border:1px solid black}");
//    pushButton->setStyleSheet("border:0;background-color:#4E4E4E;#0ff:white;font-size:16px;font-weight:blod");
    pushButton->setStyleSheet("border:0;background-color:#4E4E4E;color:#0ff;font-size:25px;font-weight:blod;width:100px");
    pushButton_2->setStyleSheet("border:0;background-color:#4E4E4E;color:white;font-size:16px;font-weight:blod");
    pushButton_3->setStyleSheet("border:0;background-color:#4E4E4E;color:white;font-size:16px;font-weight:blod");
    pushButton_7->setStyleSheet("background-color:transparent;color:white;font-size:30px;font-weight:blod");
    pushButton_8->setStyleSheet("background-color:transparent;color:white;font-size:30px;font-weight:blod");
    pushButton_6->setStyleSheet("background-color:transparent;color:white;font-size:30px;font-weight:blod");
    scrollArea->setObjectName("scrollarea");

    label_7->setStyleSheet("background-color:#393939;color:#0ff;font-size:16px;font-weight:blod");
    label_10->setStyleSheet("background-color:#393939;color:#0ff;font-size:16px;font-weight:blod");
    label_11->setStyleSheet("background-color:#393939;color:#0ff;font-size:16px;font-weight:blod");
    bgGraphicView->setStyleSheet("border:0;color:white");

    bgGraphicView->hide();
    bgLanguage->hide();
    bgGraphicColors->hide();
    tabWidget->setCurrentIndex(0);
    scrollAreaWidgetContents->setMinimumHeight(540);
    scrollAreaWidgetContents_2->setMinimumHeight(540);
    bgLanguage->setStyleSheet("border:0;color:white");
    bgGraphicColors->setStyleSheet("border:0;color:white");
    groupBox_6->setStyleSheet("background-color:#393939;border:0");
    groupBox_7->setStyleSheet("background-color:#393939;border:0");
    groupBox_8->setStyleSheet("background-color:#393939;border:0");
    scrollArea->verticalScrollBar()->setStyleSheet("QScrollBar:vertical{margin:0;width:10px;border-width:0px}\
                                QScrollBar::sub-page:vertical{background-color:#707070;border:0;}\
                                QScrollBar::add-page:vertical{background-color:#707070;border:0;}\
                                 QScrollBar::handle:vertical{margin:0;border:0;background:#292929}\
                                 QScrollBar::sub-line:vertical{image:none;}\
                                 QScrollBar::add-line:vertical{image:none;}");

    scrollArea->widget()->setStyleSheet("QWidget{background-color:#4E4E4E;border:0}"\
                              "QComboBox{background-color:#393939;border:1px solid black;color:#0ff}");
            scrollArea->setStyleSheet("QScrollArea{border:0}");

    connect(pushButton,SIGNAL(clicked()),this,SLOT(tab1Slot()));
    connect(pushButton_2,SIGNAL(clicked()),this,SLOT(tab2Slot()));
    connect(pushButton_3,SIGNAL(clicked()),this,SLOT(tab3Slot()));
    connect(pushButton_6,SIGNAL(clicked()),this,SLOT(label1Slot()));
    connect(pushButton_7,SIGNAL(clicked()),this,SLOT(label2Slot()));
    connect(pushButton_8,SIGNAL(clicked()),this,SLOT(label3Slot()));
    tab_2->setStyleSheet("color:white");
    lePathFonts->setStyleSheet("color:#0ff;background-color:#393939;border:0");
    lePathHatch->setStyleSheet("color:#0ff;background-color:#393939;border:0");
    lePathLibrary->setStyleSheet("color:#0ff;background-color:#393939;border:0");
    lePathTranslations->setStyleSheet("color:#0ff;background-color:#393939;border:0");
    leTemplate->setStyleSheet("color:#0ff;background-color:#393939;border:0");
    variablefile_field->setStyleSheet("color:#0ff;background-color:#393939;border:0");
    fonts_button->setStyleSheet("background-color:#393939");
    btTemplate->setStyleSheet("background-color:#393939");
    variablefile_button->setStyleSheet("background-color:#393939");
    label_5->setStyleSheet("background-color:#303030;border:1px solid black");
    buttonBox->button(QDialogButtonBox::Ok)->setStyleSheet("background-color:#4E4E4E;color:#0ff;width:80px;height:25px;font-size:18px");
    buttonBox->button(QDialogButtonBox::Cancel)->setStyleSheet("background-color:#4E4E4E;color:#0ff;width:80px;height:25px;font-size:18px");
    buttonBox->button(QDialogButtonBox::Cancel)->setText("取消");
    buttonBox->button(QDialogButtonBox::Ok)->setText("确定");
//    buttonBox->setStyleSheet("background-color:#4E4E4E;color:#0ff");
    scrollArea_2->verticalScrollBar()->setStyleSheet("QScrollBar:vertical{margin:0;width:10px;border-width:0px}\
                                QScrollBar::sub-page:vertical{background-color:#707070;border:0;}\
                                QScrollBar::add-page:vertical{background-color:#707070;border:0;}\
                                 QScrollBar::handle:vertical{margin:0;border:0;background:#292929}\
                                 QScrollBar::sub-line:vertical{image:none;}\
                                 QScrollBar::add-line:vertical{image:none;}");
    scrollArea_2->widget()->setStyleSheet("QWidget{background-color:#4E4E4E;border:0}"\
                              "QComboBox{background-color:#393939;border:1px solid black;color:#0ff}");
            scrollArea_2->setStyleSheet("QScrollArea{border:0}");

    groupBox_9->setStyleSheet("background-color:#393939;border:0");
    groupBox_10->setStyleSheet("background-color:#393939;border:0");
    groupBox_11->setStyleSheet("background-color:#393939;border:0");
    groupBox_12->setStyleSheet("background-color:#393939;border:0");
    groupBox_13->setStyleSheet("background-color:#393939;border:0");
    groupBox_14->setStyleSheet("background-color:#393939;border:0");
    label_8->setStyleSheet("background-color:#393939;color:#0ff;font-size:16px;font-weight:blod");
    label_9->setStyleSheet("background-color:#393939;color:#0ff;font-size:16px;font-weight:blod");
    label_12->setStyleSheet("background-color:#393939;color:#0ff;font-size:16px;font-weight:blod");
    label_13->setStyleSheet("background-color:#393939;color:#0ff;font-size:16px;font-weight:blod");
    label_14->setStyleSheet("background-color:#393939;color:#0ff;font-size:16px;font-weight:blod");
    label_15->setStyleSheet("background-color:#393939;color:#0ff;font-size:16px;font-weight:blod");
    pushButton_9->setStyleSheet("background-color:transparent;color:white;font-size:30px;font-weight:blod");
    pushButton_10->setStyleSheet("background-color:transparent;color:white;font-size:30px;font-weight:blod");
    pushButton_11->setStyleSheet("background-color:transparent;color:white;font-size:30px;font-weight:blod");
    pushButton_12->setStyleSheet("background-color:transparent;color:white;font-size:30px;font-weight:blod");
    pushButton_13->setStyleSheet("background-color:transparent;color:white;font-size:30px;font-weight:blod");
    pushButton_14->setStyleSheet("background-color:transparent;color:white;font-size:30px;font-weight:blod");
    groupBox->setStyleSheet("color:white");
    groupBox_2->setStyleSheet("color:white");
    groupBox_3->setStyleSheet("color:white");
    groupBox_4->setStyleSheet("color:white");
    groupBox_5->setStyleSheet("color:white");
    buttonGroup5->setStyleSheet("color:white");
    pb_clear_all->setStyleSheet("background-color:#4E4E4E;color:#0ff;border:1px solid black");
    pb_clear_geometry->setStyleSheet("background-color:#4E4E4E;color:#0ff;border:1px solid black");
    cbAutoSaveTime->setStyleSheet("background-color:#4E4E4E;color:#0ff;border:1px solid black");
    groupBox->hide();
    groupBox_2->hide();
    groupBox_3->hide();
    groupBox_4->hide();
    groupBox_5->hide();
    buttonGroup5->hide();
    connect(pushButton_9,SIGNAL(clicked()),this,SLOT(label4Slot()));
    connect(pushButton_10,SIGNAL(clicked()),this,SLOT(label5Slot()));
    connect(pushButton_11,SIGNAL(clicked()),this,SLOT(label6Slot()));
    connect(pushButton_12,SIGNAL(clicked()),this,SLOT(label7Slot()));
    connect(pushButton_13,SIGNAL(clicked()),this,SLOT(label8Slot()));
    connect(pushButton_14,SIGNAL(clicked()),this,SLOT(label9Slot()));
    label_16->setStyleSheet("background-color:#303030;color:#0ff;font-size:24px;font-weight:bold;border:1px solid black");
    this->setWindowFlag(Qt::FramelessWindowHint);
}

/*
 *  Destroys the object and frees any allocated resources
 */
QG_DlgOptionsGeneral::~QG_DlgOptionsGeneral()
{
    destroy();
    // no need to delete child widgets, Qt does it all for us
}

/*
 *  Sets the strings of the subwidgets using the current
 *  language.
 */
void QG_DlgOptionsGeneral::languageChange()
{
    retranslateUi(this);
}

void QG_DlgOptionsGeneral::init()
{
    // Fill combobox with languages:
    QStringList languageList = RS_SYSTEM->getLanguageList();
    languageList.sort();

    languageList.prepend("zu");
    languageList.prepend("za");
    languageList.prepend("yo");
    languageList.prepend("yi");
    languageList.prepend("xh");
    languageList.prepend("wo");
    languageList.prepend("cy");
    languageList.prepend("vo");
    languageList.prepend("vi_VN");
    languageList.prepend("ca_ES@valencia");
    languageList.prepend("uz");
    languageList.prepend("uz");
    languageList.prepend("uz");
    languageList.prepend("ur_PK");
    languageList.prepend("ur_IN");
    languageList.prepend("ur");
    languageList.prepend("uk");
    languageList.prepend("ug");
    languageList.prepend("tw");
    languageList.prepend("tk");
    languageList.prepend("tr_TR");
    languageList.prepend("ts");
    languageList.prepend("to");
    languageList.prepend("ti");
    languageList.prepend("bo");
    languageList.prepend("th_TH");
    languageList.prepend("te");
    languageList.prepend("tt");
    languageList.prepend("ta");
    languageList.prepend("tg");
    languageList.prepend("tl_PH");
    languageList.prepend("sv_FI");
    languageList.prepend("sv_SE");
    languageList.prepend("sw_KE");
    languageList.prepend("su");
    languageList.prepend("es_VE");
    languageList.prepend("es_US");
    languageList.prepend("es_UY");
    languageList.prepend("es_PR");
    languageList.prepend("es_PE");
    languageList.prepend("es_PY");
    languageList.prepend("es_PA");
    languageList.prepend("es_NI");
    languageList.prepend("es_ES");
    languageList.prepend("es_MX");
    languageList.prepend("es_HN");
    languageList.prepend("es_GT");
    languageList.prepend("es_SV");
    languageList.prepend("es_EC");
    languageList.prepend("es_DO");
    languageList.prepend("es_CR");
    languageList.prepend("es_CO");
    languageList.prepend("es_CL");
    languageList.prepend("es_BO");
    languageList.prepend("es_AR");
    languageList.prepend("es_ES");
    languageList.prepend("so");
    languageList.prepend("sl_SI");
    languageList.prepend("sk_SK");
    languageList.prepend("ss");
    languageList.prepend("si");
    languageList.prepend("sd");
    languageList.prepend("sn");
    languageList.prepend("tn");
    languageList.prepend("st");
    languageList.prepend("sh");
    languageList.prepend("sr_YU@latin");
    languageList.prepend("sr_YU");
    languageList.prepend("sr_SR@latin");
    languageList.prepend("sr_SR");
    languageList.prepend("sr_SR");
    languageList.prepend("se_NO");
    languageList.prepend("gd");
    languageList.prepend("sa");
    languageList.prepend("sg");
    languageList.prepend("ro_RO");
    languageList.prepend("sm");
    languageList.prepend("ru_RU");
    languageList.prepend("ru_UA");
    languageList.prepend("rm");
    languageList.prepend("qu");
    languageList.prepend("pa");
    languageList.prepend("pt_BR");
    languageList.prepend("pt_PT");
    languageList.prepend("pl_PL");
    languageList.prepend("ps");
    languageList.prepend("om");
    languageList.prepend("or");
    languageList.prepend("oc");
    languageList.prepend("nn_NO");
    languageList.prepend("nb_NO");
    languageList.prepend("na");
    languageList.prepend("ne_IN");
    languageList.prepend("ne_NP");
    languageList.prepend("mo");
    languageList.prepend("mn");
    languageList.prepend("mr_IN");
    languageList.prepend("mi");
    languageList.prepend("");
    languageList.prepend("mt_MT");
    languageList.prepend("ms_MY");
    languageList.prepend("ms_BN");
    languageList.prepend("ml");
    languageList.prepend("ms_MY");
    languageList.prepend("mg");
    languageList.prepend("mk_MK");
    languageList.prepend("lt_LT");
    languageList.prepend("ln");
    languageList.prepend("lv_LV");
    languageList.prepend("la");
    languageList.prepend("lo");
    languageList.prepend("ku_TR");
    languageList.prepend("ko_KR");
    languageList.prepend("");
    languageList.prepend("rn");
    languageList.prepend("ky");
    languageList.prepend("rw");
    languageList.prepend("kw_GB");
    languageList.prepend("kk");
    languageList.prepend("ks_IN");
    languageList.prepend("ks");
    languageList.prepend("kn");
    languageList.prepend("jw");
    languageList.prepend("ja_JP");
    languageList.prepend("it_CH");
    languageList.prepend("it_IT");
    languageList.prepend("ga_IE");
    languageList.prepend("ik");
    languageList.prepend("iu");
    languageList.prepend("ie");
    languageList.prepend("ia");
    languageList.prepend("id_ID");
    languageList.prepend("is_IS");
    languageList.prepend("hu_HU");
    languageList.prepend("hi_IN");
    languageList.prepend("he_IL");
    languageList.prepend("ha");
    languageList.prepend("gu");
    languageList.prepend("gn");
    languageList.prepend("kl_GL");
    languageList.prepend("el_GR");
    languageList.prepend("de_CH");
    languageList.prepend("de_LU");
    languageList.prepend("de_LI");
    languageList.prepend("de_BE");
    languageList.prepend("de_AT");
    languageList.prepend("de_DE");
    languageList.prepend("ka_GE");
    languageList.prepend("gl_ES");
    languageList.prepend("fy");
    languageList.prepend("fr_CH");
    languageList.prepend("fr_MC");
    languageList.prepend("fr_LU");
    languageList.prepend("fr_CA");
    languageList.prepend("fr_BE");
    languageList.prepend("fr_FR");
    languageList.prepend("fi_FI");
    languageList.prepend("fj");
    languageList.prepend("fa_IR");
    languageList.prepend("fo_FO");
    languageList.prepend("et_EE");
    languageList.prepend("eo");
    languageList.prepend("en_ZW");
    languageList.prepend("en_TT");
    languageList.prepend("en_ZA");
    languageList.prepend("en_PH");
    languageList.prepend("en_NZ");
    languageList.prepend("en_JM");
    languageList.prepend("en_IE");
    languageList.prepend("en_DK");
    languageList.prepend("en_CB");
    languageList.prepend("en_CA");
    languageList.prepend("en_BW");
    languageList.prepend("en_BZ");
    languageList.prepend("en_AU");
    languageList.prepend("en_US");
    languageList.prepend("en_GB");
    languageList.prepend("en_GB");
    languageList.prepend("nl_BE");
    languageList.prepend("nl_NL");
    languageList.prepend("da_DK");
    languageList.prepend("cs_CZ");
    languageList.prepend("hr_HR");
    languageList.prepend("co");
    languageList.prepend("zh_TW");
    languageList.prepend("zh_SG");
    languageList.prepend("zh_MO");
    languageList.prepend("zh_HK");
    languageList.prepend("zh_TW");
    languageList.prepend("zh_TW");
    languageList.prepend("zh_CN");
    languageList.prepend("ca_ES");
    languageList.prepend("km");
    languageList.prepend("my");
    languageList.prepend("bg_BG");
    languageList.prepend("br");
    languageList.prepend("bi");
    languageList.prepend("bh");
    languageList.prepend("dz");
    languageList.prepend("bn");
    languageList.prepend("be_BY");
    languageList.prepend("eu_ES");
    languageList.prepend("ba");
    languageList.prepend("az");
    languageList.prepend("az");
    languageList.prepend("az");
    languageList.prepend("ay");
    languageList.prepend("as");
    languageList.prepend("hy");
    languageList.prepend("ar_YE");
    languageList.prepend("ar_AE");
    languageList.prepend("ar_TN");
    languageList.prepend("ar_SY");
    languageList.prepend("ar_SD");
    languageList.prepend("ar_SA");
    languageList.prepend("ar_QA");
    languageList.prepend("ar_OM");
    languageList.prepend("ar_MA");
    languageList.prepend("ar_LY");
    languageList.prepend("ar_LB");
    languageList.prepend("ar_KW");
    languageList.prepend("ar_JO");
    languageList.prepend("ar_IQ");
    languageList.prepend("ar_EG");
    languageList.prepend("ar_BH");
    languageList.prepend("ar_DZ");
    languageList.prepend("ar");
    languageList.prepend("am");
    languageList.prepend("sq_AL");
    languageList.prepend("af_ZA");
    languageList.prepend("aa");
    languageList.prepend("ab");


	for(auto const& lang: languageList){

        RS_DEBUG->print("QG_DlgOptionsGeneral::init: adding %s to combobox",
						lang.toLatin1().data());

		QString l = RS_SYSTEM->symbolToLanguage(lang);
		if (!l.isEmpty() && cbLanguage->findData(lang)==-1) {
            RS_DEBUG->print("QG_DlgOptionsGeneral::init: %s", l.toLatin1().data());
			cbLanguage->addItem(l,lang);
			cbLanguageCmd->addItem(l, lang);
        }
    }

    RS_SETTINGS->beginGroup("/Appearance");

    // set current language:
    QString def_lang = "zh_CN";

    QString lang = RS_SETTINGS->readEntry("/Language", def_lang);
    cbLanguage->setCurrentIndex( cbLanguage->findText(RS_SYSTEM->symbolToLanguage(lang)) );

    QString langCmd = RS_SETTINGS->readEntry("/LanguageCmd", def_lang);
    cbLanguageCmd->setCurrentIndex( cbLanguageCmd->findText(RS_SYSTEM->symbolToLanguage(langCmd)) );

    // graphic view:

    // Snap Indicators
    bool indicator_lines_state = RS_SETTINGS->readNumEntry("/indicator_lines_state", 1);
    indicator_lines_checkbox->setChecked(indicator_lines_state);

    QString indicator_lines_type = RS_SETTINGS->readEntry("/indicator_lines_type", "Crosshair");
    int index = indicator_lines_combobox->findText(indicator_lines_type);
    indicator_lines_combobox->setCurrentIndex(index);

    bool indicator_shape_state = RS_SETTINGS->readNumEntry("/indicator_shape_state", 1);
    indicator_shape_checkbox->setChecked(indicator_shape_state);

    QString indicator_shape_type = RS_SETTINGS->readEntry("/indicator_shape_type", "Circle");
    index = indicator_shape_combobox->findText(indicator_shape_type);
    indicator_shape_combobox->setCurrentIndex(index);

    bool cursor_hiding = RS_SETTINGS->readNumEntry("/cursor_hiding", 0);
    cursor_hiding_checkbox->setChecked(cursor_hiding);
    
    // scale grid:
    QString scaleGrid = RS_SETTINGS->readEntry("/ScaleGrid", "1");
    cbScaleGrid->setChecked(scaleGrid=="1");
    QString minGridSpacing = RS_SETTINGS->readEntry("/MinGridSpacing", "10");
    cbMinGridSpacing->setCurrentIndex( cbMinGridSpacing->findText(minGridSpacing) );

    int checked = RS_SETTINGS->readNumEntry("/Antialiasing");
    cb_antialiasing->setChecked(checked?true:false);

    checked = RS_SETTINGS->readNumEntry("/ScrollBars");
    scrollbars_check_box->setChecked(checked?true:false);

    // preview:
	initComboBox(cbMaxPreview, RS_SETTINGS->readEntry("/MaxPreview", "100"));

    RS_SETTINGS->endGroup();

    RS_SETTINGS->beginGroup("Colors");
    initComboBox(cbBackgroundColor, RS_SETTINGS->readEntry("/background", Colors::background));
    initComboBox(cbGridColor, RS_SETTINGS->readEntry("/grid", Colors::grid));
    initComboBox(cbMetaGridColor, RS_SETTINGS->readEntry("/meta_grid", Colors::meta_grid));
    initComboBox(cbSelectedColor, RS_SETTINGS->readEntry("/select", Colors::select));
    initComboBox(cbHighlightedColor, RS_SETTINGS->readEntry("/highlight", Colors::highlight));
    initComboBox(cbStartHandleColor, RS_SETTINGS->readEntry("/start_handle", Colors::start_handle));
    initComboBox(cbHandleColor, RS_SETTINGS->readEntry("/handle", Colors::handle));
    initComboBox(cbEndHandleColor, RS_SETTINGS->readEntry("/end_handle", Colors::end_handle));
    initComboBox(cb_snap_color, RS_SETTINGS->readEntry("/snap_indicator", Colors::snap_indicator));
    RS_SETTINGS->endGroup();

    RS_SETTINGS->beginGroup("/Paths");

    lePathTranslations->setText(RS_SETTINGS->readEntry("/Translations", ""));
    lePathHatch->setText(RS_SETTINGS->readEntry("/Patterns", ""));
    lePathFonts->setText(RS_SETTINGS->readEntry("/Fonts", ""));
    lePathLibrary->setText(RS_SETTINGS->readEntry("/Library", "").trimmed());
    leTemplate->setText(RS_SETTINGS->readEntry("/Template", "").trimmed());
    variablefile_field->setText(RS_SETTINGS->readEntry("/VariableFile", "").trimmed());
    RS_SETTINGS->endGroup();

    // units:
    for (int i=RS2::None; i<RS2::LastUnit; i++) {
        if (i!=(int)RS2::None)
            cbUnit->addItem(RS_Units::unitToString((RS2::Unit)i));
    }
    // RVT_PORT cbUnit->listBox()->sort();
    cbUnit->insertItem( 0, RS_Units::unitToString(RS2::None) );

    QString def_unit = "Millimeter";

    RS_SETTINGS->beginGroup("/Defaults");
//    cbUnit->setCurrentIndex( cbUnit->findText(QObject::tr( RS_SETTINGS->readEntry("/Unit", def_unit) )) );
    cbUnit->setCurrentIndex( cbUnit->findText(QObject::tr( RS_SETTINGS->readEntry("/Unit", def_unit).toUtf8().data() )) );
    // Auto save timer
    cbAutoSaveTime->setValue(RS_SETTINGS->readNumEntry("/AutoSaveTime", 5));
    cbAutoBackup->setChecked(RS_SETTINGS->readNumEntry("/AutoBackupDocument", 1));
    cbUseQtFileOpenDialog->setChecked(RS_SETTINGS->readNumEntry("/UseQtFileOpenDialog", 1));
    cbWheelScrollInvertH->setChecked(RS_SETTINGS->readNumEntry("/WheelScrollInvertH", 0));
    cbWheelScrollInvertV->setChecked(RS_SETTINGS->readNumEntry("/WheelScrollInvertV", 0));
    cbInvertZoomDirection->setChecked(RS_SETTINGS->readNumEntry("/InvertZoomDirection", 0));
    RS_SETTINGS->endGroup();

	//update entities to selected entities to the current active layer
	RS_SETTINGS->beginGroup("/Modify");
	auto toActive=RS_SETTINGS->readNumEntry("/ModifyEntitiesToActiveLayer", 0);
	cbToActiveLayer->setChecked(toActive==1);
	RS_SETTINGS->writeEntry("/ModifyEntitiesToActiveLayer", cbToActiveLayer->isChecked()?1:0);
	RS_SETTINGS->endGroup();

	RS_SETTINGS->beginGroup("/CADPreferences");
	cbAutoZoomDrawing->setChecked(RS_SETTINGS->readNumEntry("/AutoZoomDrawing"));
	RS_SETTINGS->endGroup();

    RS_SETTINGS->beginGroup("Startup");
    cbSplash->setChecked(RS_SETTINGS->readNumEntry("/ShowSplash",1)==1);
    tab_mode_check_box->setChecked(RS_SETTINGS->readNumEntry("/TabMode", 0));
    maximize_checkbox->setChecked(RS_SETTINGS->readNumEntry("/Maximize", 0));
    left_sidebar_checkbox->setChecked(RS_SETTINGS->readNumEntry("/EnableLeftSidebar", 1));
    cad_toolbars_checkbox->setChecked(RS_SETTINGS->readNumEntry("/EnableCADToolbars", 1));
    RS_SETTINGS->endGroup();

	cbEvaluateOnSpace->setChecked(RS_SETTINGS->readNumEntry("/Keyboard/EvaluateCommandOnSpace", false));
	cbToggleFreeSnapOnSpace->setChecked(RS_SETTINGS->readNumEntry("/Keyboard/ToggleFreeSnapOnSpace", false));

    restartNeeded = false;
}

void QG_DlgOptionsGeneral::initComboBox(QComboBox* cb, QString text) {
	int idx = cb->findText(text);
	if( idx < 0) {
		idx =0;
		cb->insertItem(idx, text);
	}
	cb->setCurrentIndex( idx );
}

void QG_DlgOptionsGeneral::destroy() {
}

void QG_DlgOptionsGeneral::setRestartNeeded() {
    restartNeeded = true;
}

void QG_DlgOptionsGeneral::setTemplateFile() {
    RS2::FormatType type = RS2::FormatDXFRW;
    QG_FileDialog dlg(this);
    QString fileName = dlg.getOpenFile(&type);
    leTemplate->setText(fileName);
}

void QG_DlgOptionsGeneral::ok()
{
    if (RS_Settings::save_is_allowed)
    {
        //RS_SYSTEM->loadTranslation(cbLanguage->currentText());
        RS_SETTINGS->beginGroup("/Appearance");
        RS_SETTINGS->writeEntry("/ScaleGrid", QString("%1").arg((int)cbScaleGrid->isChecked()));
        RS_SETTINGS->writeEntry("/MinGridSpacing", cbMinGridSpacing->currentText());
        RS_SETTINGS->writeEntry("/MaxPreview", cbMaxPreview->currentText());
        RS_SETTINGS->writeEntry("/Language",cbLanguage->itemData(cbLanguage->currentIndex()));
        RS_SETTINGS->writeEntry("/LanguageCmd",cbLanguageCmd->itemData(cbLanguageCmd->currentIndex()));
        RS_SETTINGS->writeEntry("/indicator_lines_state", indicator_lines_checkbox->isChecked());
        RS_SETTINGS->writeEntry("/indicator_lines_type", indicator_lines_combobox->currentText());
        RS_SETTINGS->writeEntry("/indicator_shape_state", indicator_shape_checkbox->isChecked());      
        RS_SETTINGS->writeEntry("/indicator_shape_type", indicator_shape_combobox->currentText());
        RS_SETTINGS->writeEntry("/cursor_hiding", cursor_hiding_checkbox->isChecked());
        RS_SETTINGS->writeEntry("/Antialiasing", cb_antialiasing->isChecked()?1:0);
        RS_SETTINGS->writeEntry("/ScrollBars", scrollbars_check_box->isChecked()?1:0);
        RS_SETTINGS->endGroup();

        RS_SETTINGS->beginGroup("Colors");
        RS_SETTINGS->writeEntry("/background", cbBackgroundColor->currentText());
        RS_SETTINGS->writeEntry("/grid", cbGridColor->currentText());
        RS_SETTINGS->writeEntry("/meta_grid", cbMetaGridColor->currentText());
        RS_SETTINGS->writeEntry("/select", cbSelectedColor->currentText());
        RS_SETTINGS->writeEntry("/highlight", cbHighlightedColor->currentText());
        RS_SETTINGS->writeEntry("/start_handle", cbStartHandleColor->currentText());
        RS_SETTINGS->writeEntry("/handle", cbHandleColor->currentText());
        RS_SETTINGS->writeEntry("/end_handle", cbEndHandleColor->currentText());
        RS_SETTINGS->writeEntry("/snap_indicator", cb_snap_color->currentText());
        RS_SETTINGS->endGroup();

        RS_SETTINGS->beginGroup("/Paths");
        RS_SETTINGS->writeEntry("/Translations", lePathTranslations->text());
        RS_SETTINGS->writeEntry("/Patterns", lePathHatch->text());
        RS_SETTINGS->writeEntry("/Fonts", lePathFonts->text());
        RS_SETTINGS->writeEntry("/Library", lePathLibrary->text());
        RS_SETTINGS->writeEntry("/Template", leTemplate->text());
        RS_SETTINGS->writeEntry("/VariableFile", variablefile_field->text());
        RS_SETTINGS->endGroup();

        RS_SETTINGS->beginGroup("/Defaults");
        RS_SETTINGS->writeEntry("/Unit",
            RS_Units::unitToString( RS_Units::stringToUnit( cbUnit->currentText() ), false/*untr.*/) );
        RS_SETTINGS->writeEntry("/AutoSaveTime", cbAutoSaveTime->value() );
        RS_SETTINGS->writeEntry("/AutoBackupDocument", cbAutoBackup->isChecked() ? 1 : 0);
        RS_SETTINGS->writeEntry("/UseQtFileOpenDialog", cbUseQtFileOpenDialog->isChecked() ? 1 : 0);
        RS_SETTINGS->writeEntry("/WheelScrollInvertH", cbWheelScrollInvertH->isChecked() ? 1 : 0);
        RS_SETTINGS->writeEntry("/WheelScrollInvertV", cbWheelScrollInvertV->isChecked() ? 1 : 0);
        RS_SETTINGS->writeEntry("/InvertZoomDirection", cbInvertZoomDirection->isChecked() ? 1 : 0);
        RS_SETTINGS->endGroup();

        //update entities to selected entities to the current active layer
        RS_SETTINGS->beginGroup("/Modify");
        RS_SETTINGS->writeEntry("/ModifyEntitiesToActiveLayer", cbToActiveLayer->isChecked()?1:0);
        RS_SETTINGS->endGroup();

		RS_SETTINGS->beginGroup("/CADPreferences");
		RS_SETTINGS->writeEntry("/AutoZoomDrawing", cbAutoZoomDrawing->isChecked() ? 1 : 0);
		RS_SETTINGS->endGroup();

        RS_SETTINGS->beginGroup("Startup");
        RS_SETTINGS->writeEntry("/ShowSplash", cbSplash->isChecked()?1:0);
        RS_SETTINGS->writeEntry("/TabMode", tab_mode_check_box->isChecked()?1:0);
        RS_SETTINGS->writeEntry("/Maximize", maximize_checkbox->isChecked()?1:0);
        RS_SETTINGS->writeEntry("/EnableLeftSidebar", left_sidebar_checkbox->isChecked()?1:0);
        RS_SETTINGS->writeEntry("/EnableCADToolbars", cad_toolbars_checkbox->isChecked()?1:0);
        RS_SETTINGS->endGroup();

		RS_SETTINGS->writeEntry("/Keyboard/EvaluateCommandOnSpace", cbEvaluateOnSpace->isChecked() ? 1 : 0);
		RS_SETTINGS->writeEntry("/Keyboard/ToggleFreeSnapOnSpace", cbToggleFreeSnapOnSpace->isChecked() ? 1 : 0);
    }
	
	if (restartNeeded==true) {
        QMessageBox::warning( this, tr("Preferences"),
                              tr("Please restart the application to apply all changes."),
                              QMessageBox::Ok,
                              Qt::NoButton);
    }
    accept();
}


void QG_DlgOptionsGeneral::on_tabWidget_currentChanged(int index)
{
    current_tab = index;
}

void QG_DlgOptionsGeneral::set_color(QComboBox* combo, QColor custom)
{
    QColor current;
    current.setNamedColor(combo->lineEdit()->text());

    QColorDialog dlg;
	dlg.setCustomColor(0, custom.rgb());

    QColor color = dlg.getColor(current, this, "Select Color", QColorDialog::DontUseNativeDialog);
    if (color.isValid())
    {
        combo->lineEdit()->setText(color.name());
    }
}

void QG_DlgOptionsGeneral::on_pb_background_clicked()
{
    set_color(cbBackgroundColor, QColor(Colors::background));
}

void QG_DlgOptionsGeneral::on_pb_grid_clicked()
{
    set_color(cbGridColor, QColor(Colors::grid));
}

void QG_DlgOptionsGeneral::on_pb_meta_clicked()
{
    set_color(cbMetaGridColor, QColor(Colors::meta_grid));
}

void QG_DlgOptionsGeneral::on_pb_selected_clicked()
{
    set_color(cbSelectedColor, QColor(Colors::select));
}

void QG_DlgOptionsGeneral::on_pb_highlighted_clicked()
{
    set_color(cbHighlightedColor, QColor(Colors::highlight));
}

void QG_DlgOptionsGeneral::on_pb_start_clicked()
{
    set_color(cbStartHandleColor, QColor(Colors::start_handle));
}

void QG_DlgOptionsGeneral::on_pb_handle_clicked()
{
    set_color(cbHandleColor, QColor(Colors::handle));
}

void QG_DlgOptionsGeneral::on_pb_end_clicked()
{
    set_color(cbEndHandleColor, QColor(Colors::end_handle));
}

void QG_DlgOptionsGeneral::on_pb_snap_color_clicked()
{
    set_color(cb_snap_color, QColor(Colors::snap_indicator));
}

void QG_DlgOptionsGeneral::on_pb_clear_all_clicked()
{
    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(this, tr("Clear settings"),
                                tr("This will also include custom menus and toolbars. Continue?"),
                                QMessageBox::Yes|QMessageBox::No);
    if (reply == QMessageBox::Yes)
    {
      RS_SETTINGS->clear_all();
      QMessageBox::information(this, "info", "You must restart LibreCAD to see the changes.");
    }
}

void QG_DlgOptionsGeneral::on_pb_clear_geometry_clicked()
{
    RS_SETTINGS->clear_geometry();
    QMessageBox::information(this, "info", "You must restart LibreCAD to see the changes.");
}

void QG_DlgOptionsGeneral::setVariableFile()
{
    QString path = QFileDialog::getOpenFileName(this);
    if (!path.isEmpty())
    {
        variablefile_field->setText(QDir::toNativeSeparators(path));
    }
}

/*!
 * \brief slot for the font folder selection icon
 * \author ravas
 * \date 2016-286
 */
void QG_DlgOptionsGeneral::setFontsFolder()
{
    QFileDialog dlg(this);
    dlg.setFileMode(QFileDialog::Directory);
    dlg.setOption(QFileDialog::ShowDirsOnly);

    if (dlg.exec())
    {
        auto dir = dlg.selectedFiles()[0];
        lePathFonts->setText(QDir::toNativeSeparators(dir));
    }
}

void QG_DlgOptionsGeneral::tab1Slot()
{
    tabWidget->setCurrentIndex(0);
    pushButton->setStyleSheet("border:0;background-color:#4E4E4E;color:#0ff;font-size:25px;font-weight:blod;width:100px");
    pushButton_2->setStyleSheet("border:0;background-color:#4E4E4E;color:white;font-size:16px;font-weight:blod");
    pushButton_3->setStyleSheet("border:0;background-color:#4E4E4E;color:white;font-size:16px;font-weight:blod");
}

void QG_DlgOptionsGeneral::tab2Slot()
{
    tabWidget->setCurrentIndex(1);
    pushButton_2->setStyleSheet("border:0;background-color:#4E4E4E;color:#0ff;font-size:25px;font-weight:blod;width:100px");
    pushButton->setStyleSheet("border:0;background-color:#4E4E4E;color:white;font-size:16px;font-weight:blod");
    pushButton_3->setStyleSheet("border:0;background-color:#4E4E4E;color:white;font-size:16px;font-weight:blod");
}

void QG_DlgOptionsGeneral::tab3Slot()
{
    tabWidget->setCurrentIndex(2);
    pushButton_3->setStyleSheet("border:0;background-color:#4E4E4E;color:#0ff;font-size:25px;font-weight:blod;width:100px");
    pushButton->setStyleSheet("border:0;background-color:#4E4E4E;color:white;font-size:16px;font-weight:blod");
    pushButton_2->setStyleSheet("border:0;background-color:#4E4E4E;color:white;font-size:16px;font-weight:blod");
}

void QG_DlgOptionsGeneral::label1Slot()
{
    if(bgGraphicView->isVisible()){
        bgGraphicView->hide();
        pushButton_6->setText("+");
    }else{
        bgGraphicView->show();
        pushButton_6->setText("-");
    }
    if(bgGraphicView->isVisible()&&bgGraphicColors->isVisible()){
        scrollAreaWidgetContents->setMinimumHeight(950);
    }else{
        scrollAreaWidgetContents->setMinimumHeight(540);
    }
}

void QG_DlgOptionsGeneral::label2Slot()
{
    if(bgGraphicColors->isVisible()){
        bgGraphicColors->hide();
        pushButton_7->setText("+");
    }else{
        bgGraphicColors->show();
        pushButton_7->setText("-");
    }
    if(bgGraphicView->isVisible()&&bgGraphicColors->isVisible()){
        scrollAreaWidgetContents->setMinimumHeight(950);
    }else{
        scrollAreaWidgetContents->setMinimumHeight(540);
    }
}

void QG_DlgOptionsGeneral::label3Slot()
{
    if(bgLanguage->isVisible()){
        bgLanguage->hide();
        pushButton_8->setText("+");
    }else{
        bgLanguage->show();
        pushButton_8->setText("-");
    }
    if(bgGraphicView->isVisible()&&bgGraphicColors->isVisible()){
        scrollAreaWidgetContents->setMinimumHeight(950);
    }else{
        scrollAreaWidgetContents->setMinimumHeight(540);
    }
}

void QG_DlgOptionsGeneral::label4Slot()
{
//    groupBox->hide();
//    groupBox_2->hide();
//    groupBox_3->hide();
//    groupBox_4->hide();
//    groupBox_5->hide();
    if(buttonGroup5->isVisible()){
        buttonGroup5->hide();
        pushButton_9->setText("+");
        totalheight-=buttonGroup5->height();
    }else{
        buttonGroup5->show();
        pushButton_9->setText("-");
        totalheight+=buttonGroup5->height();
    }
    qDebug()<<buttonGroup5->height();
    qDebug()<<totalheight;
    scrollAreaWidgetContents_2->setMinimumHeight(totalheight+300);

}

void QG_DlgOptionsGeneral::label5Slot()
{
    if(groupBox_3->isVisible()){
        groupBox_3->hide();
        pushButton_10->setText("+");
        totalheight-=groupBox_3->height();
    }else{
        groupBox_3->show();
        pushButton_10->setText("-");
        totalheight+=groupBox_3->height();
    }
    scrollAreaWidgetContents_2->setMinimumHeight(totalheight+300);
}

void QG_DlgOptionsGeneral::label6Slot()
{
    if(groupBox->isVisible()){
        groupBox->hide();
        pushButton_11->setText("+");
        totalheight-=groupBox->height();
    }else{
        groupBox->show();
        pushButton_11->setText("-");
        totalheight+=groupBox->height();
    }
    scrollAreaWidgetContents_2->setMinimumHeight(totalheight+300);
}

void QG_DlgOptionsGeneral::label7Slot()
{
    if(groupBox_2->isVisible()){
        groupBox_2->hide();
        pushButton_12->setText("+");
        totalheight-=groupBox_2->height();
    }else{
        groupBox_2->show();
        pushButton_12->setText("-");
        totalheight+=groupBox_2->height();
    }
    scrollAreaWidgetContents_2->setMinimumHeight(totalheight+300);
}

void QG_DlgOptionsGeneral::label8Slot()
{
    if(groupBox_4->isVisible()){
        groupBox_4->hide();
        pushButton_13->setText("+");
        totalheight-=groupBox_4->height();
    }else{
        groupBox_4->show();
        pushButton_13->setText("-");
        totalheight+=groupBox_4->height();
    }
    scrollAreaWidgetContents_2->setMinimumHeight(totalheight+300);

}

void QG_DlgOptionsGeneral::label9Slot()
{
    if(groupBox_5->isVisible()){
        groupBox_5->hide();
        pushButton_14->setText("+");
        totalheight-=groupBox_5->height();
    }else{
        groupBox_5->show();
        pushButton_14->setText("-");
        totalheight+=groupBox_5->height();
    }
    scrollAreaWidgetContents_2->setMinimumHeight(totalheight+300);
}
