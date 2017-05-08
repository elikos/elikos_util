#include "elikos_remote_calib/gui/CalibDetectionWidget.h"
#include "elikos_remote_calib/model/PolarCoords.h"
#include <QColor>


namespace remote_calib{

SimpleSliderTextUpdater::SimpleSliderTextUpdater(QObject* parent, QSlider* sld, QLineEdit* led)
    : QObject(parent)
    , sld_(sld)
    , led_(led)
{
    connect(sld_, SIGNAL(valueChanged(int)), this, SLOT(valChangedSld(int)));
    connect(led_, SIGNAL(editingFinished()), this, SLOT(valChangedTxt()));
}
void SimpleSliderTextUpdater::setValue(int newValue){
    value_ = newValue;
    sld_->blockSignals(true);
    sld_->setValue(value_);
    sld_->blockSignals(false);
    led_->setText(QString::number(value_));
}
void SimpleSliderTextUpdater::valChangedSld(int newValue){
    value_ = newValue;
    led_->setText(QString::number(value_));
    emit valueChanged(value_);
}
void SimpleSliderTextUpdater::valChangedTxt(){
    bool ok;
    int tmp = led_->text().toInt(&ok);
    value_ = ok ? tmp  : value_;
    led_->setText(QString::number(value_));
    sld_->setValue(value_);
    emit valueChanged(value_);
}

/******************************************************************************
* Constructeur.
*
* @param parent     [in,out] le parent du widget Qt
* @param nodeName       [in] le nom du noeud auquel on se connecte
******************************************************************************/
CalibDetectionWidget::CalibDetectionWidget(QWidget* parent, const std::string& nodeName)
    : NodeCalibWidget(parent, nodeName)
    , currentStatus_()
    , currentColor_(&currentStatus_.red)
    , redPreview_(2, 1, QImage::Format_RGB888)
    , greenPreview_(2, 1, QImage::Format_RGB888)
    , whitePreview_(2, 1, QImage::Format_RGB888)
{

    ui_.setupUi(NodeCalibWidget::getPanelParent());

    deltaUpdater_     = new SimpleSliderTextUpdater(this, ui_.sldErrorDelta, ui_.txtErrorDelta);
    colorUpdaters_[0] = new SimpleSliderTextUpdater(this, ui_.sldHMax, ui_.txtHMax);
    colorUpdaters_[1] = new SimpleSliderTextUpdater(this, ui_.sldHMin, ui_.txtHMin);
    colorUpdaters_[2] = new SimpleSliderTextUpdater(this, ui_.sldSMax, ui_.txtSMax);
    colorUpdaters_[3] = new SimpleSliderTextUpdater(this, ui_.sldSMin, ui_.txtSMin);
    colorUpdaters_[4] = new SimpleSliderTextUpdater(this, ui_.sldVMax, ui_.txtVMax);
    colorUpdaters_[5] = new SimpleSliderTextUpdater(this, ui_.sldVMin, ui_.txtVMin);
    colorUpdaters_[6] = new SimpleSliderTextUpdater(this, ui_.sldPreErode, ui_.txtPreErode);
    colorUpdaters_[7] = new SimpleSliderTextUpdater(this, ui_.sldDilate, ui_.txtDilate);
    colorUpdaters_[8] = new SimpleSliderTextUpdater(this, ui_.sldPostErode, ui_.txtPostErode);

    connect(colorUpdaters_[0], SIGNAL(valueChanged(int)), this, SLOT(updateHMax(int)));
    connect(colorUpdaters_[1], SIGNAL(valueChanged(int)), this, SLOT(updateHMin(int)));
    connect(colorUpdaters_[2], SIGNAL(valueChanged(int)), this, SLOT(updateSMax(int)));
    connect(colorUpdaters_[3], SIGNAL(valueChanged(int)), this, SLOT(updateSMin(int)));
    connect(colorUpdaters_[4], SIGNAL(valueChanged(int)), this, SLOT(updateVMax(int)));
    connect(colorUpdaters_[5], SIGNAL(valueChanged(int)), this, SLOT(updateVMin(int)));
    connect(colorUpdaters_[6], SIGNAL(valueChanged(int)), this, SLOT(updatePreErode(int)));
    connect(colorUpdaters_[7], SIGNAL(valueChanged(int)), this, SLOT(updateDilate(int)));
    connect(colorUpdaters_[8], SIGNAL(valueChanged(int)), this, SLOT(updatePostErode(int)));

    ui_.image_out->setNodeHandle(targetNodeHandle_);
    ui_.image_out->setListenTopic("elikos_remote_calib/input", "compressed");

    ui_.image_calib->setNodeHandle(targetNodeHandle_);
    ui_.image_calib->setListenTopic("elikos_remote_calib/input", "compressed");

    connect(ui_.image_calib, SIGNAL(mouseLeft(float, float)), this, SLOT(processClick(float, float)));
    connect(ui_.btnTogglePlay, SIGNAL(clicked()), ui_.image_calib, SLOT(togglePaused()));

    connect(deltaUpdater_, SIGNAL(valueChanged(int)), this, SLOT(updateErrorDelta(int)));

    connect(ui_.colRed, SIGNAL(toggled(bool)),this, SLOT(updateSelectedColor()));
    connect(ui_.colGreen, SIGNAL(toggled(bool)),this, SLOT(updateSelectedColor()));
    connect(ui_.colWhite, SIGNAL(toggled(bool)),this, SLOT(updateSelectedColor()));

    updateSelectedColor();
    updateColorPreview();
}
CalibDetectionWidget::~CalibDetectionWidget(){
}

/******************************************************************************
* Processe ce click sur une image (auto-calibration)
*
* @param xNorm      [in] la coordonée en x du click normalisé (0,1) 
* @param yNorm      [in] la coordonée en y du click normalisé (0,1)
******************************************************************************/
void CalibDetectionWidget::processClick(float xNorm, float yNorm)
{
    unsigned r = 0, g = 0, b = 0;

    float dx = ui_.image_calib->getNormalizedPixelWidth();
    float dy = ui_.image_calib->getNormalizedPixelHeight();

    for(float i = xNorm - 2 * dx; i <= xNorm + 2*dx; i += dx){
        for(float j = yNorm - 2 * dy; j <= yNorm + 2*dy; j += dy){
            cv::Vec3b vec = this->ui_.image_calib->getPixel(i,j);
            r += vec[0];
            g += vec[1];
            b += vec[2];
        }
    }
    r /= 25;
    g /= 25;
    b /= 25;
    

    cv::Vec3b rgbResultat(
        (unsigned char) r,
        (unsigned char) g,
        (unsigned char) b
    );

    cv::Vec3b hsvResultat = convertVec3b(rgbResultat, CV_RGB2HSV);
    

    int minH = (int)(hsvResultat[0] - errorDeltaMultiplierH * currentDelta_) % 180;
    int maxH = (int)(hsvResultat[0] + errorDeltaMultiplierH * currentDelta_) % 180;
    minH = minH < 0 ? minH + 180 : minH;
    maxH = maxH < 0 ? maxH + 180 : maxH;

    currentColor_->col.min.h = minH;
    currentColor_->col.max.h = maxH;

    int svValues[4];
    svValues[0] = (int)(hsvResultat[1] - errorDeltaMultiplierS * currentDelta_);
    svValues[1] = (int)(hsvResultat[1] + errorDeltaMultiplierS * currentDelta_);

    svValues[2] = (int)(hsvResultat[2] - errorDeltaMultiplierV * currentDelta_);
    svValues[3] = (int)(hsvResultat[2] + errorDeltaMultiplierV * currentDelta_);

    for (unsigned i = 0; i < 4; ++i)
    {
        if(svValues[i] < 0) {
            svValues[i] = 0;
        } else if(svValues[i] > 255) {
            svValues[i] = 255;
        }
    }

    currentColor_->col.min.s = svValues[0];
    currentColor_->col.max.s = svValues[1];

    currentColor_->col.min.v = svValues[2];
    currentColor_->col.max.v = svValues[3];
    
    sendStatus();
}

/******************************************************************************
* Chande le delta d'erreur. n'a pas d'effet immédiat.
******************************************************************************/
void CalibDetectionWidget::updateErrorDelta(int newValue){
    currentDelta_ = (unsigned char)newValue;//On tronque
}

/******************************************************************************
* Change la couleur sélectionnée (par exemple, lorsqu'on clique sur un bouton.)
******************************************************************************/
void CalibDetectionWidget::updateSelectedColor()
{
    if (ui_.colGreen->isChecked()) {
        currentColor_ = &currentStatus_.green;
    } else if (ui_.colWhite->isChecked()) {
        currentColor_ = &currentStatus_.white;
    } else {//Rouge
        currentColor_ = &currentStatus_.red;
    }
    updateColorBars();
}

/******************************************************************************
* Changement des paramètres de couleurs
******************************************************************************/
void CalibDetectionWidget::updateHMax(int newVal){currentColor_->col.max.h=newVal;sendStatus();}
void CalibDetectionWidget::updateHMin(int newVal){currentColor_->col.min.h=newVal;sendStatus();}
void CalibDetectionWidget::updateSMax(int newVal){currentColor_->col.max.s=newVal;sendStatus();}
void CalibDetectionWidget::updateSMin(int newVal){currentColor_->col.min.s=newVal;sendStatus();}
void CalibDetectionWidget::updateVMax(int newVal){currentColor_->col.max.v=newVal;sendStatus();}
void CalibDetectionWidget::updateVMin(int newVal){currentColor_->col.min.v=newVal;sendStatus();}
void CalibDetectionWidget::updatePreErode(int newVal){currentColor_->deflate = newVal;sendStatus();}
void CalibDetectionWidget::updateDilate(int newVal){currentColor_->inflate = newVal;sendStatus();}
void CalibDetectionWidget::updatePostErode(int newVal){currentColor_->postDeflate = newVal;sendStatus();}//FIXME

/******************************************************************************
* Le callback de la calibration.
******************************************************************************/
void CalibDetectionWidget::calibCallback(const boost::shared_ptr< elikos_remote_calib_client::CalibDetection const > & msg)
{
    currentStatus_.red = msg->red;
    currentStatus_.green = msg->green;
    currentStatus_.white = msg->white;
    needsUpdate_ = true;
}

/******************************************************************************
* Update le gui
******************************************************************************/
void CalibDetectionWidget::update()
{
    updateColorPreview();
    updateColorBars();
}

/******************************************************************************
* Met a jour les couleurs des selecteurs de couleurs à partir de la 
* calibration en ce moment.
******************************************************************************/
void CalibDetectionWidget::updateColorPreview()
{
    QImage* images[3] = {&redPreview_, &greenPreview_, &whitePreview_};
    elikos_remote_calib_client::ColorDetectionInfo* colors[3] = {&currentStatus_.red, &currentStatus_.green, &currentStatus_.white};
    QLabel* labels[3] = {ui_.lblColRed, ui_.lblColGreen, ui_.lblColWhite};


    for(int i = 0; i < 3; ++i){
        cv::Vec3b maxColor = convertVec3b(cv::Vec3b(colors[i]->col.max.h, colors[i]->col.max.s, colors[i]->col.max.v), CV_HSV2RGB);
        cv::Vec3b minColor = convertVec3b(cv::Vec3b(colors[i]->col.min.h, colors[i]->col.min.s, colors[i]->col.min.v), CV_HSV2RGB);

        images[i]->setPixel(0, 0, qRgb(maxColor[0], maxColor[1], maxColor[2]));
        images[i]->setPixel(1, 0, qRgb(minColor[0], minColor[1], minColor[2]));
        
        labels[i]->setPixmap(QPixmap::fromImage(*(images[i])).scaled(labels[i]->size(), Qt::IgnoreAspectRatio));
        //labels[i]->setPixmap(QPixmap::fromImage(*(images[i])));

    }
}

/******************************************************************************
* Met a jour l'état des barred d'ajustement pour la couleur donnée.
******************************************************************************/
void CalibDetectionWidget::updateColorBars()
{
    colorUpdaters_[0]->setValue(currentColor_->col.max.h);
    colorUpdaters_[1]->setValue(currentColor_->col.min.h);
    colorUpdaters_[2]->setValue(currentColor_->col.max.s);
    colorUpdaters_[3]->setValue(currentColor_->col.min.s);
    colorUpdaters_[4]->setValue(currentColor_->col.max.v);
    colorUpdaters_[5]->setValue(currentColor_->col.min.v);
    colorUpdaters_[6]->setValue(currentColor_->deflate);
    colorUpdaters_[7]->setValue(currentColor_->inflate);
    colorUpdaters_[8]->setValue(currentColor_->postDeflate);
}

/******************************************************************************
* Envoi l'état de calibration en ce moment.
******************************************************************************/
void CalibDetectionWidget::sendStatus()
{
    messageManager_.sendCalibraitonMessage(currentStatus_);
}


}//end namespace
