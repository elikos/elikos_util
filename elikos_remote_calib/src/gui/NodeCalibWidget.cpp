#include "elikos_remote_calib/gui/NodeCalibWidget.h"


#include <iostream>

/*******************************************************************************
* Classe qui permet de calibrer
*******************************************************************************/
NodeCalibWidget::NodeCalibWidget(QWidget* parent)
    : QWidget(parent)
{
    ui_.setupUi(this);

    QObject::connect(ui_.btnRefresh, SIGNAL(clicked()), this, SLOT(getCalibrationFiles()));
    QObject::connect(ui_.btnSave, SIGNAL(clicked()), this, SLOT(save()));
    QObject::connect(ui_.btnLoad, SIGNAL(clicked()), this, SLOT(load()));
}

void NodeCalibWidget::save()
{
    std::cout << "save" << std::endl;
}

void NodeCalibWidget::load()
{
    std::cout << "load" << std::endl;

}

void NodeCalibWidget::getCalibrationFiles()
{
    std::cout << "refresh" << std::endl;

}

/*******************************************************************************
* Retourne une référence vers un QWidget vide placé au centre de tout. Ce widget
* devrait être le parent de tout de l'utilitaire de calibration. Cela est 
* faisable en applant ui_.setupUi(getPanelParent()); par exemple.
* @return le widget parent
*******************************************************************************/
QWidget* NodeCalibWidget::getPanelParent()
{
    return ui_.mainPanelParent;
}