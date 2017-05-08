#include "elikos_remote_calib/gui/NodeCalibWidget.h"


#include <iostream>
#include <algorithm>
#include <QMessageBox>

/*******************************************************************************
* Classe qui permet de calibrer
*
* @param parent     [in] le parent QT de cet objet
* @param nodeName   [in] le nom du noeud auquel se connecter
*******************************************************************************/
NodeCalibWidgetBase::NodeCalibWidgetBase(QWidget* parent, const std::string& nodeName)
    : QWidget(parent)
    , calibrationFileManager_(nodeName)
    , targetNodeHandle_(nodeName)
{
    ui_.setupUi(this);

    QObject::connect(ui_.btnRefresh, SIGNAL(clicked()), this, SLOT(getCalibrationFiles()));
    QObject::connect(ui_.btnSave, SIGNAL(clicked()), this, SLOT(save()));
    QObject::connect(ui_.btnLoad, SIGNAL(clicked()), this, SLOT(load()));
    QObject::connect(ui_.btnDelete, SIGNAL(clicked()), this, SLOT(deleteFile()));
}

/*******************************************************************************
* Destructeur de la classe. Se déconnecte du noeud calibré.
*******************************************************************************/
NodeCalibWidgetBase::~NodeCalibWidgetBase()
{
    std::cerr << "Disconnect from node" << std::endl;
}

/*******************************************************************************
* Retroune le nom du noeud calibré
* 
* @return le nom du noeud calibré
*******************************************************************************/
std::string NodeCalibWidgetBase::getNodeName()
{
    return calibrationFileManager_.getNodeName();
}

/******************************************************************************
* Rafraichit le GUI du node. Utile car appelé sur le même fil d'execution que 
* l'interface graphique.
******************************************************************************/
void NodeCalibWidgetBase::updateNode()
{
    if(needsUpdate_){
        needsUpdate_ = false;
        update();
    }
}

/*******************************************************************************
* Enregistre la calibration courrante du noeud dans le fochier spécifié par la 
* liste déroulante des noms de fichers.
*******************************************************************************/
void NodeCalibWidgetBase::save()
{
    QString text = ui_.cmbFileNames->currentText();
    std::string fileName = text.toStdString();

    bool sucess = calibrationFileManager_.saveCalibration(fileName);

    if(!sucess){
        showErrorMessage(QString("Il y a eu une erreur lors de l'enregistrement de '") + QString::fromStdString(fileName) + "' :(");
    }
    getCalibrationFiles();
}

/*******************************************************************************
* Charge la calibration courrante du noeud dans le fichier spécifié par la liste
* déroulante des noms de fichiers.
*******************************************************************************/
void NodeCalibWidgetBase::load()
{
    QString text = ui_.cmbFileNames->currentText();
    std::string fileName = text.toStdString();

    bool sucess = calibrationFileManager_.loadCalibration(fileName);
    if(!sucess){
        showErrorMessage(QString("Il y a eu une erreur lors du chargement de '") + QString::fromStdString(fileName) + "' :(");
    }
    getCalibrationFiles();
}

/*******************************************************************************
* Efface le fichier de calibration donné en paramètres
*******************************************************************************/
void NodeCalibWidgetBase::deleteFile()
{
    QString text = ui_.cmbFileNames->currentText();
    std::string fileName = text.toStdString();

    QMessageBox::StandardButton btn = QMessageBox::warning(
        this,
        "Attention!",
        "Voulez-vous vraiment effacer le fichier '" + text + "'?",
        QMessageBox::Yes | QMessageBox::Cancel
    );

    if(btn != QMessageBox::Yes){
        return;
    }

    bool sucess = calibrationFileManager_.deleteCalibrationFile(fileName);

    if (!sucess) {
        showErrorMessage(QString("Il y a eu une erreur lors de l'effacement de '") + QString::fromStdString(fileName) + "' :(");
    }

    getCalibrationFiles();
}

/*******************************************************************************
* Affiche un message d'erreur à l'utilisateur.
*
* @param message    [in] le message a afficher
*******************************************************************************/
void NodeCalibWidgetBase::showErrorMessage(const QString& message)
{
    QMessageBox::warning(this, "Erreur!", message);
}

/*******************************************************************************
* Retrouve les fichiers de calibrations exsistants dans le noeud et les affiches
* dans la boîte de dialogue.
*******************************************************************************/
void NodeCalibWidgetBase::getCalibrationFiles()
{
    std::vector<std::string> calibFiles;
    calibrationFileManager_.getCalibrationFileNames(calibFiles);

    QStringList qNodeNames;

    std::transform(
        calibFiles.begin(), 
        calibFiles.end(), 
        std::back_inserter(qNodeNames),
        [](const std::string& str){
            return QString::fromStdString(str);
        }
    );

    ui_.cmbFileNames->clear();
    ui_.cmbFileNames->addItems(qNodeNames);
}

/*******************************************************************************
* Retourne une référence vers un QWidget vide placé au centre de tout. Ce widget
* devrait être le parent de tout de l'utilitaire de calibration. Cela est 
* faisable en applant ui_.setupUi(getPanelParent()); par exemple.
*
* @return le widget parent
*******************************************************************************/
QWidget* NodeCalibWidgetBase::getPanelParent()
{
    return ui_.mainPanelParent;
}

/******************************************************************************
* Met a jour les composantes du noeud.
******************************************************************************/
void NodeCalibWidgetBase::update()
{
}