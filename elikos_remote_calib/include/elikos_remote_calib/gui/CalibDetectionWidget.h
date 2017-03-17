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

#include <QWidget>
#include <elikos_remote_calib/ui_calib_detection.h>
#include <elikos_remote_calib/gui/NodeCalibWidget.h>

namespace remote_calib{

class CalibDetectionWidget : public NodeCalibWidget
{
public:
    CalibDetectionWidget(QWidget* parent);


private:
    Ui::CalibDetectionWidget ui_;
};

}//end namespace remote_calib

#endif
