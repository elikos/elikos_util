#ifndef CALIB_WIDGET_H
#define CALIB_WIDGET_H

#include <iostream>

#include <QWidget>
#include <QPushButton>
#include <QTimer>

#include "elikos_remote_calib/ui_calib_main.h"
#include "elikos_remote_calib/Observer.h"
#include "elikos_remote_calib/model/Calib.h"
#include "elikos_remote_calib/gui/CalibDetectionWidget.h"
#include "elikos_remote_calib/gui/NodeCalibWidget.h"

namespace remote_calib{



class CalibWidget : public QWidget, public Observer
{
    Q_OBJECT

public:
    CalibWidget(QWidget* parent = 0);

private slots:
    //Recalcule les noeuds pouvant être calibrés
    void refreshCalibratableNodes();
    //Calibre le noeud sélectionné en ce moment
    void calibrateSelectedNode();
    //Rafraichit tout le ui
    void autoRefresh();

public:
    //implémentation de l'observeur
    virtual void update();

private:
    Ui::CalibMainWidget ui_;
    Calib calib_;
    QTimer refreshTimer_;
    
};






}


#endif