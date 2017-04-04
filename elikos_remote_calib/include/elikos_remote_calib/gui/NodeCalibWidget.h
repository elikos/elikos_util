/*******************************************************************************
* Classe de base pour les intrerfaces graphiques de chaque noeud calibrables.
* Contient un utilitaire simple pour la sauvegarde et le changement de fichiers
* de configurations.
*******************************************************************************/
#ifndef RM_NODE_CALIB_WIDGET_H
#define RM_NODE_CALIB_WIDGET_H

#include <QWidget>

#include <string>

#include "elikos_remote_calib/ui_calib_node.h"
#include "elikos_remote_calib/model/CalibrationFileManager.h"

class NodeCalibWidget : public QWidget
{
    Q_OBJECT

public:
    NodeCalibWidget(QWidget* parent, const std::string& nodeName);
    virtual ~NodeCalibWidget();

    //retourne le nom du noeud
    std::string getNodeName();
private slots:
    //lorsqu'on clique sur sauvegarder
    void save();
    //lorsqu'on clique sur charger
    void load();
    //lorqu'on clique sur effacer
    void deleteFile();
    //lorsqu'on veut rafraichir les posibilités de fichier de configuration
    void getCalibrationFiles();
    //Affiche un message d'erreur
    void showErrorMessage(const QString& message);
protected:
    //Retourne le parent du paneau principal, pour y attacher la calibration 
    //spécifique à un noeud
    QWidget* getPanelParent();
private:
    

    Ui::CalibNodeWidget ui_;
    CalibrationFileManager calibrationFileManager_;
};

#endif
