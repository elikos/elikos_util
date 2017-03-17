#include <elikos_remote_calib/gui/CalibWidget.h>

#include <QStringList>

namespace remote_calib{

/*******************************************************************************
* Variables locales au cpp (ne sortiront pas de l'unité de compilation)
*******************************************************************************/
namespace{
    //un taux de rafraichissement de 30 fps
    const int REFRESH_RATE_MS = 1000/30;
}

/*******************************************************************************
* Constructeur par paramètres de la classe CalibWidget.
* @param parent             [in] le parent de cet objet (ce parent le détruira)
*******************************************************************************/
CalibWidget::CalibWidget(QWidget* parent)
    : QWidget(parent),
      calib_(this),
      refreshTimer_(0)
{
    ui_.setupUi(this);

    QObject::connect(ui_.btnRefresh, SIGNAL(clicked()), this, SLOT(refreshCalibratableNodes()));
    QObject::connect(ui_.btnCalibrate, SIGNAL(clicked()), this, SLOT(calibrateSelectedNode()));
    QObject::connect(&refreshTimer_, SIGNAL(timeout()), this, SLOT(autoRefresh()));

    refreshTimer_.start(REFRESH_RATE_MS);
}


/*******************************************************************************
* Permet de recalculer les noeuds ros qui sont des candidats à la calibration.
* Une fois le calcul terminé, cette méthode met à jour les champs appropriés.
*******************************************************************************/
void CalibWidget::refreshCalibratableNodes()
{
    calib_.refreshCalibratableNodes();
    
}

/*******************************************************************************
* Implémentaion de l'observaur. Dans ce cas-ci, on observe Core (Core.h). Cette
* méthode rafraichit donc tout ce qu'il y a d'important dans l'interface 
* graphique de calibration.
*******************************************************************************/
void CalibWidget::update()
{
//Liste des noeuds pouvant être calibrés
    std::vector<std::string> nodeNames;
    calib_.getCalibratableNodes(nodeNames);
    QStringList qNodeNames;

    for(std::string name : nodeNames){
        qNodeNames << QString(name.c_str());
    }

    ui_.cmbTargetSelection->clear();
    ui_.cmbTargetSelection->addItems(qNodeNames);
}

/*******************************************************************************
* Calibre le noeud qui est sélectionné en ce moment par la boîte de sélection 
* des noeuds. La calibration se fait par un outil qui n'est lancé que si le 
* noeud n'est pas en train de se faire calibrer.
*******************************************************************************/
void CalibWidget::calibrateSelectedNode()
{
    std::string nodeName = ui_.cmbTargetSelection->currentText().toStdString();
    std::cout << calib_.getCalibratableNodeType(nodeName) << std::endl;

    QWidget* calibrator;
    if(calib_.getCalibratableNodeType(nodeName) == NodeType::DETECTION){
        calibrator = new CalibDetectionWidget(ui_.tabMainCalib);
    }else{
        calibrator = new NodeCalibWidget(ui_.tabMainCalib);
    }

    ui_.tabMainCalib->addTab(calibrator, QString(nodeName.c_str()));
}

/*******************************************************************************
* Lorsque cete méthode est appelée, ros est rafraichit automatiquement, et 
* l'interface change pour refléter les changements.
*******************************************************************************/
void CalibWidget::autoRefresh()
{
    calib_.refresh();
}

}//fin namespace
