#ifndef CALIB_WIDGET_H
#define CALIB_WIDGET_H

#include <QWidget>
#include <QPushButton>

#include <elikos_remote_calib/ui_calib_main.h>
#include "Core.h"
#include <iostream>

namespace remote_calib{



class CalibWidget : public QWidget
{
    Q_OBJECT

public:
    CalibWidget(QWidget* parent = 0);

public slots:
    //Recalcule les noeuds pouvant être calibrés
    void refreshCalibratableNodes();
    void calibrateSelectedNode();

private:
    Ui::CalibMainWidget ui_;
    Core core;
    
};






}


#endif