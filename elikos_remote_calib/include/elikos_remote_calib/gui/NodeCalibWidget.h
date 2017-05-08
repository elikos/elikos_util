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

#include "elikos_remote_calib/model/CalibrationMessageManager.h"


class NodeCalibWidgetBase : public QWidget
{
    Q_OBJECT

public:
    NodeCalibWidgetBase(QWidget* parent, const std::string& nodeName);
    virtual ~NodeCalibWidgetBase();

    //retourne le nom du noeud
    std::string getNodeName();

    void updateNode();

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

    //Appelée a toutes les 30ms si needsUpdate_ est vrai.
    virtual void update();

    ros::NodeHandle targetNodeHandle_;

    volatile bool needsUpdate_ = false;
private:
    Ui::CalibNodeWidget ui_;
    CalibrationFileManager calibrationFileManager_;
};



template <typename Msg>
class NodeCalibWidget : public NodeCalibWidgetBase
{
public:
    NodeCalibWidget(QWidget* parent, const std::string& nodeName);
protected:
    virtual void calibCallback(const boost::shared_ptr< Msg const > &);
    //Sert a envoyer des messages au noeud calibré
    CalibrationMessageManager<Msg, NodeCalibWidget> messageManager_;
};


//==============================================================================
//==============================================================================
//                         Définition des templates
//==============================================================================
//==============================================================================

/******************************************************************************
* Constructeur
* @param parent     [in,out] requis par Qt. Parent de la composante.
* @param nodeName   [in]     le nom de noeud qu'on doit calibrer.
******************************************************************************/
template <typename Msg>
NodeCalibWidget<Msg>::NodeCalibWidget(QWidget* parent, const std::string& nodeName)
    : NodeCalibWidgetBase(parent, nodeName)
    , messageManager_(nodeName, &NodeCalibWidget<Msg>::calibCallback, this)
{

}

/******************************************************************************
* Méthode à surcharger pour pouvoir écouter la calibration initiale.
******************************************************************************/
template<typename Msg>
void NodeCalibWidget<Msg>::calibCallback(const boost::shared_ptr< Msg const > &)
{
}

#endif
