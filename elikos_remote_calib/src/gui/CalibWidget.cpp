#include <elikos_remote_calib/CalibWidget.h>

#include <QStringList>

namespace remote_calib{



CalibWidget::CalibWidget(QWidget* parent)
    : QWidget(parent)
{
    ui_.setupUi(this);
    QObject::connect(ui_.btnRefresh, SIGNAL(clicked()), this, SLOT(refreshCalibratableNodes()));
    QObject::connect(ui_.btnCalibrate, SIGNAL(clicked()), this, SLOT(calibrateSelectedNode()));
}


/*******************************************************************************
* Permet de recalculer les noeuds ros qui sont des candidats à la calibration.
* Une fois le calcul terminé, cette méthode met à jour les champs appropriés.
*******************************************************************************/
void CalibWidget::refreshCalibratableNodes()
{
    std::vector<std::string> nodeNames;
    core.getCalibratableNodes(nodeNames);
    QStringList qNodeNames;

    for(std::string name : nodeNames){
        qNodeNames << QString(name.c_str());
    }

    ui_.cmbTargetSelection->clear();
    ui_.cmbTargetSelection->addItems(qNodeNames);
}

void CalibWidget::calibrateSelectedNode()
{
    std::cout << ui_.cmbTargetSelection->currentText().toStdString() << std::endl;
}

}
