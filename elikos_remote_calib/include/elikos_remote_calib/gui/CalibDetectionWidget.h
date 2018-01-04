/*******************************************************************************
* Classe d'interface graphique QT
*
* Cette classe est en fait l'implémentation contenant les 'signals' et les 
* 'slots' de QT pour la composante de calibration du noeud de détection.
*
* @author Arnaud Paré-Vogt
*******************************************************************************/
#ifndef CALIB_DETECTION_WIDGET_H
#define CALIB_DETECTION_WIDGET_H

#include <memory>

#include <QWidget>

#include <elikos_remote_calib/ui_calib_detection.h>
#include <elikos_remote_calib/gui/NodeCalibWidget.h>
#include <elikos_msgs/CalibDetection.h>


namespace remote_calib{

/******************************************************************************
* Classe pour connecter le valeur d'un slider avec un text field. Emmet le 
* signal valueChanged(int) si la valeur d'un des deux change.
******************************************************************************/
class SimpleSliderTextUpdater : public QObject
{
    Q_OBJECT
public:
    SimpleSliderTextUpdater(QObject* parent, QSlider* sld, QLineEdit* led);
signals:
    void valueChanged(int value);
public slots:
    void setValue(int newValue);
private slots:
    void valChangedSld(int newValue);
    void valChangedTxt();
private:
    QSlider* sld_;
    QLineEdit* led_;
    int value_;
};

class CalibDetectionWidget : public NodeCalibWidget<elikos_msgs::CalibDetection>
{
    Q_OBJECT
public:
    CalibDetectionWidget(QWidget* parent, const std::string& nodeName);
    virtual ~CalibDetectionWidget();

private slots:
    void processClick(float xNorm, float yNorm);
    void updateErrorDelta(int newValue);

    void updateSelectedColor();

    void updateHMax(int newVal);
    void updateHMin(int newVal);
    void updateSMax(int newVal);
    void updateSMin(int newVal);
    void updateVMax(int newVal);
    void updateVMin(int newVal);
    void updatePreErode(int newVal);
    void updateDilate(int newVal);
    void updatePostErode(int newVal);

protected:
    //Ros
    virtual void calibCallback(const boost::shared_ptr< elikos_msgs::CalibDetection const > &);
    //GUI
    virtual void update();

private:
    void updateColorPreview();
    void updateColorBars();
    void sendStatus();

    Ui::CalibDetectionWidget ui_;

    QImage redPreview_;
    QImage greenPreview_;
    QImage whitePreview_;

    SimpleSliderTextUpdater* colorUpdaters_[9];//hmax hmin smax smin vmax vmin pre dil post 

    SimpleSliderTextUpdater* deltaUpdater_;
    unsigned char currentDelta_=0;
    double errorDeltaMultiplierH = 0.45;
    double errorDeltaMultiplierS = 1.1;
    double errorDeltaMultiplierV = 2;


    elikos_msgs::CalibDetection currentStatus_;
    elikos_msgs::ColorDetectionInfo* currentColor_;
};

}//end namespace remote_calib

#endif
